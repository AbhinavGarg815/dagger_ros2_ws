#!/usr/bin/env python3
"""Replay a track_record.sh bag through person_tracking.py OFFLINE and score it.

    python3 tools/replay_tracker.py ~/dagger_bags/track_ids1_<HHMMSS>
    python3 tools/replay_tracker.py <bag> --set max_missed=20 --set iou_min=0.3
    python3 tools/replay_tracker.py <bag> --live          # score what the live tracker did
    python3 tools/replay_tracker.py <bag> --reid --select-at 30   # appearance from the bag's
        UI JPEGs (OSNet engine), select the person nearest the image centre at 30 s,
        and report re-identifications

Re-computes each detection's bearing + LiDAR range exactly like person_tracker
(camera model + static TF + range_at on the scan nearest in time), feeds the
tracker frame by frame at the recorded timing, and reports ID stability:

  ids created        -- total track ids handed out (lower is better for the same scene)
  id switches        -- a detection that continues a person (IoU >= 0.3 with that
                        person's box one frame earlier) but got a different id
  short tracks       -- ids that lived < 1 s (churn)
  selected history   -- for the id the operator clicked: frames TRACKING / held / lost

No ROS nodes are started; only the bag is read. Detections below --min-score are
dropped first, so one bag recorded with det_conf:=0.2 can be replayed as if the
detector ran at 0.45.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "dagger_bringup"))
from dagger_bringup.person_tracking import Detection, Tracker, range_at, scan_xy  # noqa: E402


def quat_rotate(q, v):
    u = np.array([q[0], q[1], q[2]])
    s = q[3]
    return 2.0 * np.dot(u, v) * u + (s * s - np.dot(u, u)) * v + 2.0 * s * np.cross(u, v)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = ["/follow/detections", "/scan_safety", "/odometry/filtered", "/tf_static",
            "/d435/color/camera_info", "/follow/tracks", "/follow/select",
            "/follow/ui/image/compressed"]
    r.set_filter(__import__("rosbag2_py").StorageFilter(topics=[w for w in want if w in types]))
    out = {w: [] for w in want}
    while r.has_next():
        topic, raw, t = r.read_next()
        out[topic].append((t * 1e-9, deserialize_message(raw, get_message(types[topic]))))
    return out


def static_chain(tf_msgs, parent, child):
    """Compose static transforms child->...->parent: returns (translation, quaternion)."""
    edges = {}
    for _, m in tf_msgs:
        for t in m.transforms:
            tr, q = t.transform.translation, t.transform.rotation
            edges[t.child_frame_id] = (t.header.frame_id, np.array([tr.x, tr.y, tr.z]),
                                       (q.x, q.y, q.z, q.w))
    pos, rot, f = np.zeros(3), (0.0, 0.0, 0.0, 1.0), child
    while f != parent:
        if f not in edges:
            raise KeyError(f"no static transform from {child} up to {parent} (stuck at {f})")
        par, t, q = edges[f]
        pos = quat_rotate(q, pos) + t
        rot = qmul(q, rot)
        f = par
    return pos, rot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--set", action="append", default=[], help="Tracker kwarg=value")
    ap.add_argument("--min-score", type=float, default=0.0)
    ap.add_argument("--live", action="store_true", help="score the recorded /follow/tracks instead")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--reid", action="store_true", help="appearance from the bag's UI JPEGs")
    ap.add_argument("--engine", default="/home/jetson/ros2_ws/models/osnet_x0_25_msmt17_fp16.engine")
    ap.add_argument("--select-at", type=float, default=None,
                    help="select the ranged person nearest the image centre at this time (s)")
    ap.add_argument("--rot", type=int, default=180, help="camera mounting rotation (deg)")
    a = ap.parse_args()
    B = read_bag(os.path.expanduser(a.bag))
    dets = B["/follow/detections"]
    if not dets:
        sys.exit("no /follow/detections in the bag")
    t0 = dets[0][0]

    if a.live:
        frames = [(t, [(tr.id, (tr.x1, tr.y1, tr.x2, tr.y2), tr.visible) for tr in m.tracks],
                   m.selected_id, m.target_state) for t, m in B["/follow/tracks"]]
    else:
        info = B["/d435/color/camera_info"][0][1]
        K = (info.k[0], info.k[4], info.k[2], info.k[5])
        cam_frame = dets[0][1].header.frame_id
        cam_t, cam_q = static_chain(B["/tf_static"], "base_footprint", cam_frame)
        scans = B["/scan_safety"]
        scan_t = np.array([t for t, _ in scans])
        odom = B["/odometry/filtered"]
        odom_t = np.array([t for t, _ in odom])
        kw = {}
        for s in a.set:
            k, v = s.split("=")
            kw[k] = float(v) if "." in v else int(v)
        trk = Tracker(**kw)
        sel_events = [] if a.select_at is not None else sorted(
            (t, m.data) for t, m in B["/follow/select"])
        imgs = {(m.header.stamp.sec, m.header.stamp.nanosec): bytes(m.data)
                for _, m in B["/follow/ui/image/compressed"]} if a.reid else {}
        reid = None
        if a.reid:
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "dagger_bringup"))
            from dagger_bringup.reid_trt import ReidTRT, crop
            import cv2
            reid = ReidTRT(a.engine)
        selected_done, relinks, sel_hist = False, [], []
        frames = []
        for t, m in dets:
            while sel_events and sel_events[0][0] <= t:
                _, sid = sel_events.pop(0)
                (trk.clear() if sid < 0 else trk.select(sid))
            i = int(np.searchsorted(scan_t, t)) - 1
            sc = scans[i][1] if i >= 0 and t - scan_t[i] < 0.4 else None
            pts = scan_xy(sc.ranges, sc.angle_min, sc.angle_increment, sc.range_min,
                          sc.range_max) if sc is not None else None
            j = int(np.searchsorted(odom_t, t)) - 1
            T = None
            if j >= 0 and t - odom_t[j] < 0.5:
                p, q = odom[j][1].pose.pose.position, odom[j][1].pose.pose.orientation
                T = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            dl = []
            for d in m.detections:
                sc_ = d.results[0].hypothesis.score if d.results else 0.0
                if sc_ < a.min_score:
                    continue
                cx, cy, w, h = d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y
                ray = quat_rotate(cam_q, np.array([(cx - K[2]) / K[0], (cy - K[3]) / K[1], 1.0]))
                bearing = math.atan2(ray[1], ray[0])
                xy = None
                if pts is not None:
                    half = min(math.radians(25), max(math.radians(2), math.atan((w / 2) / K[0]) * 1.2))
                    xy = range_at(pts[0], pts[1], cam_t[:2], bearing, half, 0.6, 2)
                dl.append(Detection((cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2), sc_, bearing, xy,
                                    near=h >= 0.85 * info.height))
            key = (m.header.stamp.sec, m.header.stamp.nanosec)
            if reid is not None and key in imgs and dl:
                fr = cv2.imdecode(np.frombuffer(imgs[key], np.uint8), cv2.IMREAD_COLOR)
                Hh, Ww = fr.shape[:2]
                if a.rot == 180:
                    fr = cv2.rotate(fr, cv2.ROTATE_180)
                crops, idx = [], []
                for k, d_ in enumerate(dl):
                    x1, y1, x2, y2 = d_.box
                    ub = (Ww - x2, Hh - y2, Ww - x1, Hh - y1) if a.rot == 180 else (x1, y1, x2, y2)
                    c = crop(fr, ub) if d_.score >= 0.3 else None
                    if c is not None:
                        crops.append(c)
                        idx.append(k)
                for k, e in zip(idx, reid.embed(crops)):
                    dl[k].emb = e
            tracks, _ = trk.update(dl, t, T, pts)
            if a.select_at is not None and not selected_done and t - t0 >= a.select_at:
                cand = [x for x in tracks if x.visible and x.ever_ranged]
                if cand:
                    pick = min(cand, key=lambda x: abs((x.box[0] + x.box[2]) / 2 - info.width / 2))
                    trk.select(pick.id)
                    selected_done = True
                    print(f"  selected #{pick.id} at {t - t0:.1f} s (range {pick.range:.1f} m)")
            if trk.relinked is not None and (not relinks or relinks[-1] != trk.relinked):
                relinks.append(trk.relinked)
                print(f"  RE-IDENTIFIED #{trk.selected_id} at {t - t0:.1f} s "
                      f"(was new track #{trk.relinked[1]}, appearance {trk.relinked[2]:.2f})")
            if trk.selected_id >= 0:
                if not sel_hist or sel_hist[-1][1] != trk.state():
                    sel_hist.append((t - t0, trk.state()))
            frames.append((t, [(x.id, tuple(x.box), x.visible) for x in tracks],
                           trk.selected_id, trk.state()))
        if sel_hist:
            print("  selected-person state changes: " + ", ".join(f"{s_} @{tt:.1f}s" for tt, s_ in sel_hist))

    # ---- score ----
    births, last_seen, switches, prev = {}, {}, 0, []
    states = {}
    for t, tr, sel, st in frames:
        vis = [(i, b) for i, b, v in tr if v]
        for i, b in vis:
            births.setdefault(i, t)
            last_seen[i] = t
            # continuation of someone visible last frame, under a different id?
            for pi, pb in prev:
                if pi != i:
                    x1, y1 = max(b[0], pb[0]), max(b[1], pb[1])
                    x2, y2 = min(b[2], pb[2]), min(b[3], pb[3])
                    inter = max(0, x2 - x1) * max(0, y2 - y1)
                    u = (b[2] - b[0]) * (b[3] - b[1]) + (pb[2] - pb[0]) * (pb[3] - pb[1]) - inter
                    if u > 0 and inter / u >= 0.3 and pi not in [k for k, _ in vis]:
                        switches += 1
        prev = vis
        if sel >= 0:
            states[st] = states.get(st, 0) + 1
    life = {i: last_seen[i] - births[i] for i in births}
    short = sum(1 for v in life.values() if v < 1.0)
    dur = frames[-1][0] - frames[0][0]
    print(f"{'live' if a.live else 'replay'} {os.path.basename(a.bag.rstrip('/'))} "
          f"{' '.join(a.set)} min_score={a.min_score}: {len(frames)} frames / {dur:.0f} s | "
          f"ids created {len(births)} | id switches {switches} | short (<1 s) {short} | "
          f"selected: {states or '-'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
