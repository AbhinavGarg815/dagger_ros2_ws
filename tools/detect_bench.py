#!/usr/bin/env python3
"""Benchmark + sanity-check the YOLOX TensorRT detector. NON-MOTION, no ROS.

    bash tools/fetch_model.sh                         # build the engine first
    python3 tools/detect_bench.py                     # sanity + 200-frame benchmark
    python3 tools/detect_bench.py --iters 500 --conf 0.3

Answers the question that gates the whole follow-me detector: what does this cost
on a 6-core Orin that Nav2 already starves at times, and are the detections real?

TWO THINGS IT CHECKS, in order:

1. CORRECTNESS. YOLOX changed its input normalisation between releases and the
   wrong choice produces plausible-looking nonsense rather than an error, so this
   runs a known image BOTH ways and prints what each finds. On the classic YOLOX
   dog.jpg the right one finds a dog, a bicycle and a truck at sane boxes; the
   wrong one finds junk or nothing. Use --legacy to pin it once it is known.

2. COST. Median/p95 milliseconds split across preprocess, inference and decode,
   plus CPU per core, GPU busy, RAM and power sampled from tegrastats over the
   run. Compare against the LiDAR's 10 Hz -- the follow loop does not need more.

The reported cost is for the CURRENT nvpmodel power mode. Raising it makes these
numbers better and leaves less headroom for everything else; that is the user's
call, not this script's.
"""
import argparse
import os
import re
import statistics
import subprocess
import sys
import time

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src", "dagger_bringup"))

COCO = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
        "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
        "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
        "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
        "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
        "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
        "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
        "toothbrush"]

TEGRA_RE = re.compile(
    r"RAM (\d+)/(\d+)MB.*?CPU \[([^\]]+)\].*?GR3D_FREQ (\d+)%.*?VDD_IN (\d+)mW")


class Tegrastats:
    """Samples tegrastats in the background for the duration of a block."""

    def __init__(self):
        self.proc = None
        self.samples = []

    def __enter__(self):
        self.proc = subprocess.Popen(["tegrastats", "--interval", "500"],
                                     stdout=subprocess.PIPE, text=True)
        return self

    def __exit__(self, *_):
        self.proc.terminate()
        try:
            out, _ = self.proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            out = ""
        for line in out.splitlines():
            m = TEGRA_RE.search(line)
            if m:
                cores = [int(c.split("%")[0]) for c in m.group(3).split(",")
                         if "%" in c and c.split("%")[0].isdigit()]
                self.samples.append({"ram": int(m.group(1)), "ram_tot": int(m.group(2)),
                                     "cores": cores, "gpu": int(m.group(4)),
                                     "mw": int(m.group(5))})

    def report(self):
        if not self.samples:
            return "  (no tegrastats samples)"
        gpu = [s["gpu"] for s in self.samples]
        mw = [s["mw"] for s in self.samples]
        ncore = len(self.samples[0]["cores"])
        busiest = [max(s["cores"]) for s in self.samples]
        total = [sum(s["cores"]) / ncore for s in self.samples]
        return (f"  GPU busy   mean {statistics.mean(gpu):5.1f} %   max {max(gpu):3d} %\n"
                f"  CPU all-{ncore}  mean {statistics.mean(total):5.1f} %   "
                f"busiest core max {max(busiest):3d} %\n"
                f"  RAM        {self.samples[-1]['ram']} / {self.samples[-1]['ram_tot']} MB"
                f"      power mean {statistics.mean(mw) / 1000:.2f} W")


def describe(dets, limit=6):
    if not dets:
        return "    (nothing)"
    return "\n".join(
        f"    {COCO[c] if c < len(COCO) else c:<14s} {s:.2f}  "
        f"[{x1:6.0f} {y1:6.0f} {x2:6.0f} {y2:6.0f}]"
        for x1, y1, x2, y2, s, c in dets[:limit])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--engine", default=os.path.join(WS, "models", "yolox_nano_fp16.engine"))
    ap.add_argument("--image", default=os.path.join(WS, "models", "dog.jpg"))
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--legacy", choices=["auto", "true", "false"], default="auto")
    a = ap.parse_args()

    import cv2
    from dagger_bringup.yolox_trt import YoloxTRT

    if not os.path.exists(a.engine):
        print(f"no engine at {a.engine} -- run: bash tools/fetch_model.sh")
        sys.exit(1)
    img = cv2.imread(a.image)
    if img is None:
        print(f"could not read {a.image}")
        sys.exit(1)
    print(f"engine {os.path.basename(a.engine)}   image {os.path.basename(a.image)} "
          f"{img.shape[1]}x{img.shape[0]}   conf {a.conf}\n")

    # --- 1. which normalisation does this checkpoint want? --------------------
    chosen = a.legacy
    if a.legacy == "auto":
        best, best_n = None, -1
        for mode in (True, False):
            d = YoloxTRT(a.engine, conf_thres=a.conf, classes=None, legacy=mode)
            dets = d.detect(img)
            print(f"  legacy={str(mode):5s} -> {len(dets)} detections")
            print(describe(dets))
            # prefer more detections, tie-break on the top score
            rank = (len(dets), dets[0][4] if dets else 0.0)
            if rank > (best_n, 0.0):
                best_n, best = len(dets), mode
            d.close()
        chosen = str(best).lower()
        print(f"\n  -> using legacy={chosen}"
              f"{'  (NEITHER found anything -- check the model/engine)' if best_n == 0 else ''}\n")

    det = YoloxTRT(a.engine, conf_thres=a.conf, classes=None, legacy=(chosen == "true"))
    print(f"input {det.input_hw[1]}x{det.input_hw[0]}   "
          f"outputs {[b[1] for b in det.outputs]}\n")

    # --- 2. cost --------------------------------------------------------------
    from dagger_bringup.yolox_trt import preprocess, decode_outputs
    for _ in range(10):                      # warm up: first runs pay lazy init
        det.detect(img)

    t_pre, t_inf, t_post, t_all = [], [], [], []
    with Tegrastats() as tg:
        t0_all = time.perf_counter()
        for _ in range(a.iters):
            s = time.perf_counter()
            chw, ratio = preprocess(img, det.input_hw, det.legacy)
            t1 = time.perf_counter()
            raw = det.infer_raw(chw)
            t2 = time.perf_counter()
            decode_outputs(np.asarray(raw, np.float32).reshape(-1, raw.shape[-1]),
                           det.input_hw)
            t3 = time.perf_counter()
            t_pre.append((t1 - s) * 1e3)
            t_inf.append((t2 - t1) * 1e3)
            t_post.append((t3 - t2) * 1e3)
            t_all.append((t3 - s) * 1e3)
        wall = time.perf_counter() - t0_all

    def stat(name, v):
        v = sorted(v)
        return (f"  {name:<12s} median {statistics.median(v):6.2f} ms   "
                f"p95 {v[int(0.95 * len(v)) - 1]:6.2f} ms   max {v[-1]:6.2f} ms")

    print(f"{a.iters} frames in {wall:.2f} s  ->  {a.iters / wall:.1f} FPS "
          f"(LiDAR runs at 10 Hz; the follow loop does not need more)\n")
    for n, v in (("preprocess", t_pre), ("inference", t_inf),
                 ("decode", t_post), ("TOTAL", t_all)):
        print(stat(n, v))
    print(f"\nload during the run (tegrastats):\n{tg.report()}")
    mode = subprocess.run(["nvpmodel", "-q"], capture_output=True, text=True).stdout
    print(f"\npower mode: {' '.join(mode.split()) or 'unknown'}")
    det.close()


if __name__ == "__main__":
    main()
