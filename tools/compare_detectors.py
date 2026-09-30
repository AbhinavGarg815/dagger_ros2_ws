#!/usr/bin/env python3
"""Compare YOLOX person detectors on a track_record.sh bag. NO ROVER.

    python3 tools/compare_detectors.py <bag> --ref 4:you --ref 7:far --ref 8:plant ...

Runs each TensorRT engine in models/ on the bag's UI JPEGs (the frames the live
detector processed; rotated 180 deg for inference exactly like person_detector)
and scores them against boxes of tracks recorded in /follow/tracks, labelled by
the operator (--ref id:label; label 'plant'/'sign'/... = a known false positive,
anything else = a real person). Per model: per-frame time (wall and CPU, whole
pipeline: preprocess + infer + decode + NMS), and per labelled track the share
of its frames with a detection overlapping it (IoU >= 0.3) at >= 0.45, and the
median best score.
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "dagger_bringup"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--ref", action="append", default=[], help="track_id:label")
    ap.add_argument("--models", default="yolox_nano,yolox_tiny,yolox_s,yolox_m")
    ap.add_argument("--every", type=int, default=2, help="use every Nth frame")
    ap.add_argument("--rot", type=int, default=180)
    a = ap.parse_args()
    FP = {"plant", "sign", "object", "fp"}
    refs = {int(r.split(":")[0]): r.split(":")[1] for r in a.ref}

    import cv2
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from dagger_bringup.person_tracking import iou_matrix
    from dagger_bringup.yolox_trt import COCO_PERSON, YoloxTRT

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=os.path.expanduser(a.bag), storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=["/follow/tracks", "/follow/ui/image/compressed"]))
    imgs, boxes, allboxes = {}, {}, {}
    while r.has_next():
        tp, raw, _ = r.read_next()
        m = deserialize_message(raw, get_message(types[tp]))
        k = (m.header.stamp.sec, m.header.stamp.nanosec)
        if tp == "/follow/tracks":
            boxes[k] = {tr.id: (tr.x1, tr.y1, tr.x2, tr.y2) for tr in m.tracks
                        if tr.visible and tr.id in refs}
            allboxes[k] = [(tr.x1, tr.y1, tr.x2, tr.y2) for tr in m.tracks]
        else:
            imgs[k] = bytes(m.data)
    keys = sorted(k for k in imgs if boxes.get(k))[::a.every]
    print(f"{len(keys)} labelled frames; refs {refs}")

    for name in a.models.split(","):
        eng = f"/home/jetson/ros2_ws/models/{name}_fp16.engine"
        if not os.path.exists(eng):
            print(f"{name}: no engine")
            continue
        det = YoloxTRT(eng, conf_thres=0.2, classes=(COCO_PERSON,))
        stats = {i: [] for i in refs}
        unexplained = 0
        wall, cpu = [], []
        for k in keys:
            fr = cv2.imdecode(np.frombuffer(imgs[k], np.uint8), cv2.IMREAD_COLOR)
            h, w = fr.shape[:2]
            t0, c0 = time.perf_counter(), time.process_time()
            up = cv2.rotate(fr, cv2.ROTATE_180) if a.rot == 180 else fr
            dets = det.detect(up)
            wall.append(time.perf_counter() - t0)
            cpu.append(time.process_time() - c0)
            if a.rot == 180:
                dets = [(w - x2, h - y2, w - x1, h - y1, s, c) for x1, y1, x2, y2, s, c in dets]
            known = allboxes.get(k, [])
            for d in dets:                   # confident, and nothing recorded there
                if d[4] >= 0.5 and (not known or iou_matrix([d[:4]], known).max() < 0.1):
                    unexplained += 1
            for tid, bx in boxes[k].items():
                if not dets:
                    stats[tid].append(0.0)
                    continue
                iou = iou_matrix([bx], [d[:4] for d in dets])[0]
                cand = [d[4] for d, v in zip(dets, iou) if v >= 0.3]
                stats[tid].append(max(cand) if cand else 0.0)
        det.close()
        line = [f"{name:11s} {1000 * np.median(wall):5.1f} ms/frame (cpu {1000 * np.median(cpu):5.1f} ms, "
                f"input {det.input_hw[0]}x{det.input_hw[1]})"]
        for tid, lab in refs.items():
            v = np.array(stats[tid])
            if not len(v):
                continue
            hit = np.mean(v >= 0.45) * 100
            line.append(f"#{tid} {lab}: {'FP ' if lab in FP else ''}{hit:3.0f}% >=0.45 "
                        f"(median {np.median(v):.2f}, n {len(v)})")
        line.append(f"unexplained >=0.5: {unexplained} in {len(keys)} frames")
        print(" | ".join(line), flush=True)


if __name__ == "__main__":
    main()
