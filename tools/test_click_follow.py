#!/usr/bin/env python3
"""Scene test for click-to-follow: the REAL person_tracker + follow_controller in a
simulated world, with a scripted operator. NO ROVER, NO FCU.

    python3 tools/test_click_follow.py main      # out of view L/R + bystander + expiry
    python3 tools/test_click_follow.py cross     # A (3 m) and B (6 m) cross in the image
    python3 tools/test_click_follow.py occlude   # A walks behind B (LiDAR + camera occluded)
    python3 tools/test_click_follow.py wall      # A out of view against a wall
    python3 tools/test_click_follow.py all

Runs in ROS_DOMAIN_ID 42 (set before rclpy is imported, inherited by the nodes it
starts), so it cannot reach a live stack on domain 0 -- a second follow_controller
on the live domain WOULD drive an armed rover.

World: people are 0.25 m circles, walls are segments; a 360 deg scan is raycast
from base_footprint, and camera detections exist for people inside the 55.7 deg
field of view who are not hidden behind a nearer person. The rover stays at the
odom origin. The operator clicks the track nearest person A's true position,
then presses FOLLOW -- exactly what the browser UI sends.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

os.environ["ROS_DOMAIN_ID"] = "42"
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import tf2_ros  # noqa: E402
from dagger_msgs.msg import PersonTracks  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo, LaserScan  # noqa: E402
from std_msgs.msg import Bool, Int32, String  # noqa: E402
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,  # noqa: E402
                             ObjectHypothesisWithPose)

FX, FY, CX, CY, W, H = 606.04, 605.56, 329.92, 259.31, 640, 480
CAM = np.array([0.2925, 0.0, 0.99])
R = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])   # base <- optical
LIB = "/home/jetson/ros2_ws/install/dagger_bringup/lib/dagger_bringup"


def q_from_R(m):
    t = m.trace(); s = math.sqrt(max(1e-9, t + 1)) * 2
    if t > 0:
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    s = math.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
    return (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)


def ray_circle(a, cx, cy, r):
    dx, dy = math.cos(a), math.sin(a); b = dx * cx + dy * cy
    c = cx * cx + cy * cy - r * r; disc = b * b - c
    if disc < 0:
        return math.inf
    t = b - math.sqrt(disc)
    return t if t > 0 else math.inf


def ray_seg(a, x1, y1, x2, y2):
    dx, dy = math.cos(a), math.sin(a); ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-9:
        return math.inf
    t = (x1 * ey - y1 * ex) / den; u = (x1 * dy - y1 * dx) / den
    return t if t > 0 and 0 <= u <= 1 else math.inf


def world(node, people, walls):
    stamp = node.get_clock().now().to_msg()
    s = LaserScan(); s.header.stamp = stamp; s.header.frame_id = "base_footprint"
    s.angle_min, s.angle_max, s.angle_increment = -math.pi, math.pi, 0.0087
    s.range_min, s.range_max = 1.0, 20.0
    rng = []
    for i in range(int(2 * math.pi / 0.0087)):
        a = -math.pi + i * 0.0087
        d = min([ray_circle(a, px, py, 0.25) for px, py in people.values()] +
                [ray_seg(a, *w) for w in walls] + [math.inf])
        rng.append(d if d <= 20.0 else math.inf)
    s.ranges = rng
    det = Detection2DArray(); det.header.stamp = stamp; det.header.frame_id = "d435_color_optical_frame"
    seen = []
    for name, (px, py) in people.items():
        v = R.T @ (np.array([px, py, 1.0]) - CAM)
        if v[2] <= 0.3:
            continue
        u = FX * v[0] / v[2] + CX
        if not (15 <= u <= W - 15):
            continue                                  # outside the field of view
        r = math.hypot(px - CAM[0], py - CAM[1])
        half = FX * math.atan2(0.25, r)
        seen.append((r, u, half, name))
    for r, u, half, name in seen:                     # hidden behind someone nearer?
        if any(r2 < r - 0.3 and abs(u2 - u) < 0.7 * (half + h2) for r2, u2, h2, _ in seen):
            continue
        d = Detection2D(); d.header = det.header; d.bbox = BoundingBox2D()
        d.bbox.center.position.x, d.bbox.center.position.y = float(u), float(CY)
        d.bbox.size_x, d.bbox.size_y = float(2 * half), float(FY * 1.7 / r)
        h = ObjectHypothesisWithPose(); h.hypothesis.class_id = "0"; h.hypothesis.score = 0.9
        d.results.append(h); det.detections.append(d)
    return s, det


def lerp(a, b, f):
    return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)


SCENES = {
    "main": ([(7.0, -8.0, 7.0, 8.0)], [
        (0, 3, lambda f: {"A": (3.0, 0.3), "B": (4.5, -1.8)}, "A ahead, B aside; click A, FOLLOW"),
        (3, 7, lambda f: {"A": lerp((3.0, 0.3), (1.8, 2.8), f), "B": (4.5, -1.8)}, "A walks out LEFT"),
        (7, 9, lambda f: {"A": lerp((1.8, 2.8), (3.0, 0.5), f), "B": (4.5, -1.8)}, "A back into view"),
        (9, 12, lambda f: {"A": lerp((3.0, 0.5), (2.0, -2.8), f), "B": lerp((4.5, -1.8), (4.0, 0.0), f)},
         "B moves ahead; A walks out RIGHT"),
        # to 25 s: A is LOST at ~15.5 s, the controller then searches for its 6 s and
        # disables at ~21.5-22 s -- ending the scene at 22 s raced that message
        (12, 25, lambda f: {"A": (2.0, -2.8), "B": (4.0, 0.0)}, "A stays out of view; B in view")]),
    "cross": ([(9.0, -8.0, 9.0, 8.0)], [
        (0, 2, lambda f: {"A": (3.0, 1.2), "B": (6.0, -1.2)}, "A left at 3 m, B right at 6 m"),
        (2, 10, lambda f: {"A": lerp((3.0, 1.2), (3.0, -1.2), f), "B": lerp((6.0, -1.2), (6.0, 1.2), f)},
         "they cross in the image"),
        (10, 12, lambda f: {"A": (3.0, -1.2), "B": (6.0, 1.2)}, "apart again")]),
    "occlude": ([(9.0, -8.0, 9.0, 8.0)], [
        (0, 2, lambda f: {"A": (4.8, 1.6), "B": (3.0, 0.0)}, "A behind-left of B"),
        (2, 6, lambda f: {"A": lerp((4.8, 1.6), (4.8, 0.0), f), "B": (3.0, 0.0)}, "A walks behind B"),
        (6, 8, lambda f: {"A": (4.8, 0.0), "B": (3.0, 0.0)}, "A hidden behind B"),
        (8, 12, lambda f: {"A": lerp((4.8, 0.0), (4.8, -1.8), f), "B": (3.0, 0.0)}, "A comes out right"),
        (12, 14, lambda f: {"A": (4.8, -1.8), "B": (3.0, 0.0)}, "A visible again")]),
    "wall": ([(7.0, -8.0, 7.0, 8.0), (0.5, 2.3, 6.0, 2.3)], [
        (0, 3, lambda f: {"A": (3.0, 0.3)}, "A ahead"),
        (3, 7, lambda f: {"A": lerp((3.0, 0.3), (2.0, 1.95), f)}, "A walks out LEFT, against a wall"),
        (7, 12, lambda f: {"A": (2.0, 1.95)}, "A stands against the wall, out of view")]),
}


def run(scen):
    walls, phases = SCENES[scen]
    procs = [subprocess.Popen([f"{LIB}/{exe}", "--ros-args", *args], start_new_session=True,
                              stdout=open(f"/tmp/click_sim_{scen}_{exe}.log", "w"),
                              stderr=subprocess.STDOUT)
             for exe, args in (("person_tracker", []), ("follow_controller", []))]
    n = Node(f"scene_{scen}")
    stb = tf2_ros.StaticTransformBroadcaster(n)
    t = TransformStamped(); t.header.frame_id, t.child_frame_id = "base_footprint", "d435_color_optical_frame"
    t.header.stamp = n.get_clock().now().to_msg()
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = CAM
    (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
     t.transform.rotation.w) = q_from_R(R)
    stb.sendTransform([t])
    be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
    p_info = n.create_publisher(CameraInfo, "/d435/color/camera_info", be)
    p_scan = n.create_publisher(LaserScan, "/scan_safety", be)
    p_det = n.create_publisher(Detection2DArray, "/follow/detections", 10)
    p_odom = n.create_publisher(Odometry, "/odometry/filtered", be)
    p_sel = n.create_publisher(Int32, "/follow/select", 10)
    p_en = n.create_publisher(Bool, "/follow/enable", 10)
    LOG, tracks = [], [None]
    T0 = [time.time()]
    n.create_subscription(PoseStamped, "/follow/target", lambda m: LOG.append(
        ("tgt", time.time() - T0[0], m.pose.position.x, m.pose.position.y)), be)
    n.create_subscription(String, "/follow/target_source", lambda m: LOG.append(
        ("src", time.time() - T0[0], m.data)), 10)
    n.create_subscription(Int32, "/follow/track_id", lambda m: LOG.append(
        ("id", time.time() - T0[0], m.data)), 10)
    n.create_subscription(String, "/follow/state", lambda m: LOG.append(
        ("state", time.time() - T0[0], m.data)), 10)
    n.create_subscription(Twist, "/cmd_vel_follow_raw", lambda m: LOG.append(
        ("cmd", time.time() - T0[0], m.linear.x, m.angular.z)), 10)
    n.create_subscription(PersonTracks, "/follow/tracks", lambda m: tracks.__setitem__(0, m), 10)
    n.create_subscription(Log, "/rosout", lambda m: LOG.append(
        ("log", time.time() - T0[0], m.name, m.msg))
        if m.name in ("person_tracker", "follow_controller") and m.level >= 20 else None, 50)
    info = CameraInfo(); info.width, info.height = W, H
    info.k = [FX, 0., CX, 0., FY, CY, 0., 0., 1.]

    def odom():
        o = Odometry(); o.header.frame_id, o.child_frame_id = "odom", "base_footprint"
        o.header.stamp = n.get_clock().now().to_msg(); o.pose.pose.orientation.w = 1.0
        p_odom.publish(o)

    n.create_timer(0.05, odom)
    end = time.time() + 6.0                            # nodes up + discovery
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

    truth, sel_id, clicked, enabled = [], None, False, False
    T0[0] = time.time()
    for t0, t1, fn, note in phases:
        LOG.append(("mark", t0, note))
        while time.time() - T0[0] < t1:
            el = time.time() - T0[0]
            people = fn((el - t0) / (t1 - t0))
            truth.append((el, people))
            scan, det = world(n, people, walls)
            p_info.publish(info); p_scan.publish(scan); p_det.publish(det)
            if not clicked and el > 1.0 and tracks[0] is not None:
                ax, ay = people["A"]
                cand = [(math.hypot(tr.position.x - ax, tr.position.y - ay), tr.id)
                        for tr in tracks[0].tracks if tr.position.x == tr.position.x]
                if cand and min(cand)[0] < 0.5:
                    sel_id = min(cand)[1]
                    for _ in range(3):
                        p_sel.publish(Int32(data=sel_id))
                    clicked = True
                    LOG.append(("mark", el, f"operator clicks #{sel_id} (A)"))
            if clicked and not enabled and el > 1.6:
                for _ in range(3):
                    p_en.publish(Bool(data=True))
                enabled = True
                LOG.append(("mark", el, "operator presses FOLLOW"))
            stop = time.time() + 0.1
            while time.time() < stop:
                rclpy.spin_once(n, timeout_sec=0.005)
    for p in procs:
        os.killpg(p.pid, signal.SIGINT)
    time.sleep(2)
    for p in procs:
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGKILL)
    n.destroy_node()
    return LOG, truth, sel_id


def at(truth, t):
    best = min(truth, key=lambda e: abs(e[0] - t))
    return best[1]


def evaluate(scen, LOG, truth, sel_id):
    tg = [(e[1], e[2], e[3]) for e in LOG if e[0] == "tgt"]
    cm = [(e[1], e[2], e[3]) for e in LOG if e[0] == "cmd"]
    src = [(e[1], e[2]) for e in LOG if e[0] == "src"]
    ids = [(e[1], e[2]) for e in LOG if e[0] == "id"]
    st = [(e[1], e[2]) for e in LOG if e[0] == "state"]
    logs = [(e[1], e[2], e[3]) for e in LOG if e[0] == "log"]
    brg = lambda x, y: abs(math.degrees(math.atan2(y, x)))  # noqa: E731
    # every published target must be A (never B): within 0.6 m of A's true position
    far_from_a = [t for t, x, y in tg if math.hypot(x - at(truth, t)["A"][0], y - at(truth, t)["A"][1]) > 0.6]
    res = [("operator selected A", sel_id is not None),
           ("every target published is person A (within 0.6 m), never anyone else",
            bool(tg) and not far_from_a, f"{len(far_from_a)} of {len(tg)} off A"),
           ("/follow/track_id is always the selected id", ids and all(i == sel_id for _, i in ids))]
    if scen == "main":
        lid = [t for t, s in src if s == "lidar"]
        res.append(("LiDAR holds A out of camera view (> 45 deg)",
                    bool(lid) and max(brg(x, y) for t, x, y in tg) > 45))
        bad = sum(1 for t, v, w in cm if v > 0.01 and any(q[0] <= t for q in tg)
                  and brg(*[q for q in tg if q[0] <= t][-1][1:]) > 36)
        res.append(("turns in place (v = 0) beyond 35 deg bearing", bad == 0, f"{bad} bad commands"))
        lost = [t for t, n_, m in logs if "LOST" in m]
        res.append(("A expires out of view -> LOST (no hand-over to B)", bool(lost)))
        dis = [t for t, s in st if s.startswith("disabled: target")]
        after = [v for t, v, w in cm if dis and t > dis[0] + 0.2 and abs(v) > 0.01]
        res.append(("controller disables itself and never drives at B", bool(dis) and not after,
                    dis and st[-1][1]))
    elif scen == "cross":
        res.append(("A keeps its id through the crossing (targets all A)", not far_from_a))
    elif scen == "occlude":
        near_b = [t for t, x, y in tg if math.hypot(x - 3.0, y) < 0.6]
        res.append(("while A is hidden behind B, B is never the target", not near_b))
        states = {s for _, s in st}
        res.append(("controller never drives toward B",
                    not any(v > 0.01 and near_b for t, v, w in cm), f"states {sorted(states)}"))
    elif scen == "wall":
        refused = [t for t, n_, m in logs if "not a person" in m]      # LOST line carries the reason
        res.append(("LiDAR hold refuses the wall-merged cluster", bool(refused)))
        res.append(("never drives forward during the whole scene", all(v <= 0.01 for t, v, w in cm)))
    print(f"\n=== {scen} ===")
    for e in LOG:
        if e[0] == "mark":
            print(f"  {e[1]:6.2f}s  -- {e[2]}")
        elif e[0] == "log":
            print(f"  {e[1]:6.2f}s  {e[2]}: {e[3][:100]}")
    for item in res:
        name, ok = item[0], item[1]
        detail = item[2] if len(item) > 2 else ""
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail and not ok else ""))
    return [ok for _, ok, *_ in res]


def main():
    scen = sys.argv[1] if len(sys.argv) > 1 else "main"
    rclpy.init()
    allres = []
    for s in (SCENES if scen == "all" else [scen]):
        allres += evaluate(s, *run(s))
    rclpy.try_shutdown()
    print(f"\n{sum(allres)}/{len(allres)} passed", flush=True)
    os._exit(0 if allres and all(allres) else 1)


if __name__ == "__main__":
    main()
