#!/usr/bin/env python3
"""Which ArUco marker is in front of the camera? READ-ONLY, no motion.

    python3 tools/marker_probe.py            # watch for 8 s
    python3 tools/marker_probe.py --secs 15 --size 0.15

Needs T1 with use_camera:=true (and T2 for the LiDAR size estimate). Hold the
marker 1.5-3 m in front of the rover, facing the camera, and keep it still.

Tries every predefined ArUco dictionary on each frame and reports, per marker:
the dictionary and id, how large it appears, and -- from the LiDAR range at its
bearing -- an estimate of its printed size. Ends with the launch arguments for
follow.launch.py. With --size it instead compares the range computed from that
size with the LiDAR's.

A marker printed from DICT_4X4_50 also decodes in DICT_4X4_100/250/1000 (they
start with the same ids), so the SMALLEST dictionary that reads it is reported:
fewer ids = fewer false decodes.
"""
import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "dagger_bringup"))

DICTS = ["DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
         "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
         "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
         "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000",
         "DICT_ARUCO_ORIGINAL"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=8.0)
    ap.add_argument("--size", type=float, default=0.0, help="printed side (m) to check against the LiDAR")
    ap.add_argument("--image", default="/d435/color/image_raw")
    a = ap.parse_args()

    import cv2
    import rclpy
    import tf2_ros
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from tf2_msgs.msg import TFMessage
    from dagger_bringup.marker_tracking import aruco_detector, marker_pose, quat_to_R
    from dagger_bringup.person_tracking import range_at, scan_xy

    rclpy.init()
    n = Node("marker_probe")
    be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
    st = {"img": None, "K": None, "D": None, "scan": None, "frame": None}
    buf = tf2_ros.Buffer()
    bridge = CvBridge()

    def on_img(m):
        st["img"], st["frame"] = bridge.imgmsg_to_cv2(m, desired_encoding="bgr8"), m.header.frame_id

    def on_tf(m):
        for t in m.transforms:
            buf.set_transform_static(t, "tf_static")

    n.create_subscription(Image, a.image, on_img, be)
    n.create_subscription(CameraInfo, a.image.rsplit("/", 1)[0] + "/camera_info",
                          lambda m: st.update(K=(m.k[0], m.k[4], m.k[2], m.k[5]), D=list(m.d)), be)
    n.create_subscription(LaserScan, "/scan_safety", lambda m: st.update(scan=m), be)
    n.create_subscription(TFMessage, "/tf_static", on_tf,
                          QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL))
    dets = {d: aruco_detector(d) for d in DICTS}
    seen = {}                       # (dict, id) -> [side px, LiDAR depth or None, pnp range, lidar range]
    frames, end = 0, time.time() + a.secs
    print(f"watching {a.image} for {a.secs:.0f} s -- hold the marker still, facing the camera ...")
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if st["img"] is None:
            continue
        img, st["img"] = st["img"], None
        frames += 1
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cam = None
        if st["K"] is not None:
            try:
                tf = buf.lookup_transform("base_footprint", st["frame"], rclpy.time.Time())
                t, q = tf.transform.translation, tf.transform.rotation
                cam = (np.array([t.x, t.y, t.z]), quat_to_R((q.x, q.y, q.z, q.w)))
            except tf2_ros.TransformException:
                pass
        pts = None
        if st["scan"] is not None:
            s = st["scan"]
            pts = scan_xy(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
        for name, det in dets.items():
            for mid, c in det(gray):
                c = np.array(c).reshape(4, 2)
                sides = np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1)
                depth = r_pnp = r_lidar = None
                if cam is not None:
                    fx, fy, cx, cy = st["K"]
                    u, v = c.mean(axis=0)
                    d = cam[1] @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
                    if pts is not None and len(pts[0]):
                        half = max(math.radians(1.0), math.atan(sides.mean() / 2 / fx) * 1.5)
                        xl = range_at(pts[0], pts[1], (cam[0][0], cam[0][1]), math.atan2(d[1], d[0]),
                                      half, 0.5, 2, reject_extended=False)
                        if xl is not None:
                            r_lidar = math.hypot(xl[0] - cam[0][0], xl[1] - cam[0][1])
                            depth = r_lidar / math.hypot(d[0], d[1])     # optical-frame Z
                    if a.size > 0:
                        pose = marker_pose(c.ravel(), a.size, st["K"], st["D"])
                        if pose is not None:
                            p = cam[1] @ pose[0] + cam[0]
                            r_pnp = math.hypot(p[0] - cam[0][0], p[1] - cam[0][1])
                seen.setdefault((name, mid), []).append((float(sides.max()), depth, r_pnp, r_lidar))
    n.destroy_node()
    rclpy.shutdown()

    if not frames:
        sys.exit(f"no frames on {a.image} -- is T1 up with use_camera:=true?")
    if not seen:
        sys.exit(f"{frames} frames, no ArUco marker decoded in any dictionary. Closer (1.5-3 m), "
                 f"flat, facing the camera, out of direct glare; it needs a white margin around "
                 f"the black square.")
    # one physical marker reads in several dictionaries of the same bit size: keep the smallest
    best = {}
    for (name, mid), v in seen.items():
        bits = name.split("_")[1]
        k = (bits, mid)
        if k not in best or DICTS.index(name) < DICTS.index(best[k][0]):
            # a real marker held still reads in nearly every frame; background
            # texture gave a 4X4_1000 id in 26 % of the frames of a static scene
            # (live, 2026-09-30), always small
            if len(v) >= max(3, 0.5 * frames) and np.median([x[0] for x in v]) >= 15.0:
                best[k] = (name, mid, v)
    if not best:
        top = sorted(seen.items(), key=lambda kv: -len(kv[1]))[:3]
        sys.exit(f"{frames} frames; no marker read steadily. Stray decodes (background texture, or a "
                 f"marker too far / moving): "
                 + ", ".join(f"{k[0]} id {k[1]} in {len(v)} frames, {np.median([x[0] for x in v]):.0f} px"
                             for k, v in top)
                 + ". Hold the marker still, 1.5-3 m away, facing the camera.")
    print(f"\n{frames} frames. Markers read in at least half of them (>= 15 px):")
    for name, mid, v in sorted(best.values(), key=lambda b: -len(b[2])):
        side = np.median([x[0] for x in v])
        line = f"  {name}  id {mid}: {len(v)}/{frames} frames, {side:.0f} px"
        fx = st["K"][0] if st["K"] else None
        depths = [x[1] for x in v if x[1] is not None]
        size = None
        if depths and fx:
            size = float(np.median([x[0] * x[1] / fx for x in v if x[1] is not None]))
            line += f", LiDAR {np.median([x[3] for x in v if x[3] is not None]):.2f} m -> printed side ~ {size:.3f} m"
        else:
            line += ", no LiDAR return at its bearing (size not estimated)"
        print(line)
        if a.size > 0:
            both = [(x[2], x[3]) for x in v if x[2] is not None and x[3] is not None]
            if both:
                d = np.array([p - l for p, l in both])
                print(f"      with size {a.size:.3f} m: marker range - LiDAR range = "
                      f"{np.median(d):+.2f} m (spread {np.std(d):.2f})")
        use = a.size if a.size > 0 else size
        print(f"      -> ros2 launch dagger_bringup follow.launch.py marker_dictionary:={name} "
              f"marker_ids:={mid} marker_size:=" + (f"{use:.3f}" if use else "<measure it>"))
    print("\nMeasure the black square with a ruler: the LiDAR estimate includes whatever the marker "
          "is mounted on\n(a hand-held sheet reads the body behind it). A 1 cm error on a 15 cm "
          "marker is 7 % in range.")


if __name__ == "__main__":
    main()
