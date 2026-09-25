#!/usr/bin/env python3
"""YOLOX inference on TensorRT, with numpy pre/post-processing. No PyTorch.

torch, torchvision, ultralytics and onnxruntime are not installed on this Jetson
and installing torch on JetPack is painful, so the detector is: a pre-exported
YOLOX ONNX -> a TensorRT engine built by trtexec (tools/fetch_model.sh) -> this
module, which runs it through the TensorRT Python API with cv2 for the resize.

Device memory comes from cuda-python (`pip install --user cuda-python==12.6.*`,
matching CUDA 12.6). TensorRT's Python API cannot allocate device buffers on its
own and pycuda is not installed either, so without cuda-python none of this runs.

PREPROCESSING IS VERSION-SENSITIVE. YOLOX changed its input normalisation: older
weights ("legacy") want BGR->RGB, /255 and ImageNet mean/std; newer ones want raw
0-255 BGR. Feeding the wrong one yields plausible-looking garbage rather than an
error, so `legacy` is a parameter and tools/detect_bench.py determines it
empirically against a known image instead of trusting a guess.

MEASURED 2026-09-25 for the yolox_nano.onnx in the Megvii 0.1.1rc0 release:
legacy=False is correct. On the reference dog.jpg it finds dog 0.86 / bicycle
0.80 / car 0.80 at sane boxes; legacy=True finds NOTHING at all. Hence the
default below. Re-run detect_bench.py --legacy auto if the model ever changes.
"""
import numpy as np

try:
    import cv2
except ImportError as e:  # pragma: no cover
    raise ImportError("yolox_trt needs cv2 (python3-opencv)") from e

_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

COCO_PERSON = 0


def _cuda_check(ret):
    """cuda-python returns (err, value...) tuples; raise on a non-success err."""
    from cuda import cudart
    err, *rest = ret if isinstance(ret, tuple) else (ret,)
    if err != cudart.cudaError_t.cudaSuccess:
        raise RuntimeError(f"CUDA error: {err}")
    return rest[0] if len(rest) == 1 else tuple(rest)


def preprocess(bgr, input_hw, legacy=False):
    """Letterbox into input_hw, padding bottom/right with 114. Returns (CHW float32, ratio)."""
    ih, iw = input_hw
    padded = np.full((ih, iw, 3), 114, dtype=np.uint8)
    r = min(ih / bgr.shape[0], iw / bgr.shape[1])
    nh, nw = int(bgr.shape[0] * r), int(bgr.shape[1] * r)
    padded[:nh, :nw] = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)

    img = padded.astype(np.float32)
    if legacy:
        img = img[:, :, ::-1]          # BGR -> RGB
        img /= 255.0
        img -= _MEAN
        img /= _STD
    return np.ascontiguousarray(img.transpose(2, 0, 1)), r


def decode_outputs(raw, input_hw, p6=False):
    """YOLOX heads emit grid-relative boxes; apply the grid offsets and strides.

    raw: (N, 85) = [cx, cy, w, h, objectness, 80 class scores], grid-relative.
    """
    strides = [8, 16, 32, 64] if p6 else [8, 16, 32]
    grids, expanded = [], []
    for s in strides:
        hs, ws = input_hw[0] // s, input_hw[1] // s
        xv, yv = np.meshgrid(np.arange(ws), np.arange(hs))
        grid = np.stack((xv, yv), 2).reshape(-1, 2)
        grids.append(grid)
        expanded.append(np.full((grid.shape[0], 1), s))
    grids = np.concatenate(grids, 0).astype(np.float32)
    expanded = np.concatenate(expanded, 0).astype(np.float32)

    out = raw.copy()
    out[:, :2] = (out[:, :2] + grids) * expanded
    out[:, 2:4] = np.exp(out[:, 2:4]) * expanded
    return out


def nms(boxes, scores, thres):
    """Plain single-class NMS. boxes: (N,4) xyxy. Returns kept indices."""
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0.0, xx2 - xx1 + 1) * np.maximum(0.0, yy2 - yy1 + 1)
        iou = inter / (areas[i] + areas[order[1:]] - inter)
        order = order[1:][iou <= thres]
    return keep


class YoloxTRT:
    """Loads a trtexec-built engine and returns detections in ORIGINAL image pixels.

    detect() -> list of (x1, y1, x2, y2, score, cls), highest score first.
    """

    def __init__(self, engine_path, conf_thres=0.35, nms_thres=0.45,
                 classes=(COCO_PERSON,), legacy=False):
        import tensorrt as trt
        from cuda import cudart

        self._cudart = cudart
        self.conf_thres = conf_thres
        self.nms_thres = nms_thres
        self.classes = None if classes is None else set(classes)
        self.legacy = legacy

        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(
                f"could not deserialize {engine_path} -- an engine is built for one "
                "GPU + TensorRT version; rebuild it with tools/fetch_model.sh")
        self.context = self.engine.create_execution_context()
        self.stream = _cuda_check(cudart.cudaStreamCreate())
        # cudart calls take the cudaStream_t object; TensorRT wants a raw handle.
        self.stream_ptr = int(self.stream)

        # TensorRT 10 addresses tensors by name, not by binding index.
        self.bindings = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.context.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            is_input = self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
            host = np.empty(shape, dtype=dtype)
            dev = _cuda_check(cudart.cudaMalloc(host.nbytes))
            self.context.set_tensor_address(name, int(dev))
            self.bindings.append((name, shape, dtype, is_input, host, dev))
            if is_input:
                self.input_hw = (shape[2], shape[3])   # NCHW

        self.outputs = [b for b in self.bindings if not b[3]]
        self.inputs = [b for b in self.bindings if b[3]]
        if len(self.inputs) != 1:
            raise RuntimeError(f"expected 1 input tensor, got {len(self.inputs)}")

    def infer_raw(self, chw):
        """Run the engine on one preprocessed CHW image. Returns the output array."""
        cudart = self._cudart
        name, shape, dtype, _, host, dev = self.inputs[0]
        np.copyto(host, chw.reshape(shape).astype(dtype, copy=False))
        _cuda_check(cudart.cudaMemcpyAsync(
            dev, host.ctypes.data, host.nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream))
        self.context.execute_async_v3(self.stream_ptr)
        for _n, _s, _d, _i, ohost, odev in self.outputs:
            _cuda_check(cudart.cudaMemcpyAsync(
                ohost.ctypes.data, odev, ohost.nbytes,
                cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, self.stream))
        _cuda_check(cudart.cudaStreamSynchronize(self.stream))
        return self.outputs[0][4]

    def detect(self, bgr):
        chw, ratio = preprocess(bgr, self.input_hw, self.legacy)
        raw = self.infer_raw(chw)
        pred = decode_outputs(np.asarray(raw, dtype=np.float32).reshape(-1, raw.shape[-1]),
                              self.input_hw)

        obj = pred[:, 4]
        cls_scores = pred[:, 5:]
        cls_id = cls_scores.argmax(1)
        score = obj * cls_scores[np.arange(cls_scores.shape[0]), cls_id]

        m = score > self.conf_thres
        if self.classes is not None:
            m &= np.isin(cls_id, list(self.classes))
        if not m.any():
            return []

        cx, cy, w, h = pred[m, 0], pred[m, 1], pred[m, 2], pred[m, 3]
        boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], 1) / ratio
        score, cls_id = score[m], cls_id[m]

        out = []
        for c in np.unique(cls_id):
            sel = cls_id == c
            idx = nms(boxes[sel], score[sel], self.nms_thres)
            for i in idx:
                b = boxes[sel][i]
                out.append((float(b[0]), float(b[1]), float(b[2]), float(b[3]),
                            float(score[sel][i]), int(c)))
        out.sort(key=lambda d: -d[4])
        return out

    def close(self):
        for _n, _s, _d, _i, _h, dev in self.bindings:
            self._cudart.cudaFree(dev)
        self._cudart.cudaStreamDestroy(self.stream)
