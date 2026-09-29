#!/usr/bin/env python3
"""Person re-identification (appearance signature) on TensorRT. No PyTorch.

OSNet x0.25 trained on MSMT17 (torchreid), pre-exported ONNX from Hugging Face
anriha/osnet_x0_25_msmt17 (sha256 e78604f4...d277), engine built by trtexec
(tools/fetch_model.sh reid). The ONNX has a FIXED batch of 16: crops are sent in
chunks of 16, zero-padded. 8.2 ms per batch on the Orin Nano (FP16, measured
2026-09-29).

Input: person crops, BGR (cv2), any size -> resized to 256x128 (h x w), RGB,
ImageNet mean/std (torchreid's test transform). Output: 512-d signature per crop,
L2-normalised, so the cosine similarity of two people is a dot product.

A signature says "looks like", never "is": it changes with pose, light and
occlusion. person_tracking.py only uses it together with position and with a
margin over everybody else in view.
"""
import numpy as np

try:
    import cv2
except ImportError as e:  # pragma: no cover
    raise ImportError("reid_trt needs cv2 (python3-opencv)") from e

from dagger_bringup.yolox_trt import _cuda_check

_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32) * 255.0
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32) * 255.0


class ReidTRT:

    def __init__(self, engine_path):
        import tensorrt as trt
        from cuda import cudart

        self._cudart = cudart
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"could not deserialize {engine_path} -- rebuild it with "
                               "tools/fetch_model.sh reid (engines are per GPU + TensorRT)")
        self.context = self.engine.create_execution_context()
        self.stream = _cuda_check(cudart.cudaStreamCreate())
        self.stream_ptr = int(self.stream)
        self.io = {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.context.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            host = np.empty(shape, dtype=dtype)
            dev = _cuda_check(cudart.cudaMalloc(host.nbytes))
            self.context.set_tensor_address(name, int(dev))
            is_in = self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
            self.io["in" if is_in else "out"] = (name, shape, host, dev)
        _, (self.batch, _, self.h, self.w), _, _ = self.io["in"]
        self.dim = self.io["out"][1][1]

    def _run(self, chunk):
        cudart = self._cudart
        _, _, ihost, idev = self.io["in"]
        _, _, ohost, odev = self.io["out"]
        ihost[:] = 0.0
        ihost[:len(chunk)] = chunk
        _cuda_check(cudart.cudaMemcpyAsync(idev, ihost.ctypes.data, ihost.nbytes,
                                           cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
                                           self.stream))
        self.context.execute_async_v3(self.stream_ptr)
        _cuda_check(cudart.cudaMemcpyAsync(ohost.ctypes.data, odev, ohost.nbytes,
                                           cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
                                           self.stream))
        _cuda_check(cudart.cudaStreamSynchronize(self.stream))
        return ohost[:len(chunk)].copy()

    def preprocess(self, crop_bgr):
        im = cv2.resize(crop_bgr, (self.w, self.h), interpolation=cv2.INTER_LINEAR)
        im = (im[:, :, ::-1].astype(np.float32) - _MEAN) / _STD      # BGR -> RGB, normalise
        return im.transpose(2, 0, 1)

    def embed(self, crops_bgr):
        """List of BGR crops -> (N, 512) L2-normalised signatures (float32)."""
        if not crops_bgr:
            return np.zeros((0, self.dim), np.float32)
        x = np.stack([self.preprocess(c) for c in crops_bgr])
        feats = np.concatenate([self._run(x[i:i + self.batch])
                                for i in range(0, len(x), self.batch)])
        return feats / np.maximum(np.linalg.norm(feats, axis=1, keepdims=True), 1e-9)

    def close(self):
        for _n, _s, _h, dev in self.io.values():
            self._cudart.cudaFree(dev)
        self._cudart.cudaStreamDestroy(self.stream)


def crop(frame, box, pad=0.05):
    """Box (x1, y1, x2, y2) -> a padded crop of frame, or None if degenerate."""
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = box
    px, py = (x2 - x1) * pad, (y2 - y1) * pad
    x1, y1 = max(0, int(x1 - px)), max(0, int(y1 - py))
    x2, y2 = min(w, int(x2 + px)), min(h, int(y2 + py))
    if x2 - x1 < 8 or y2 - y1 < 16:
        return None
    return frame[y1:y2, x1:x2]
