#!/usr/bin/env python3
"""Scene test for click-to-follow: the REAL person_tracker + follow_controller in a
simulated world, with a scripted operator. NO ROVER, NO FCU.

    python3 tools/test_click_follow.py main      # out of view L/R + bystander + expiry
    python3 tools/test_click_follow.py cross     # A (3 m) and B (6 m) cross in the image
    python3 tools/test_click_follow.py occlude   # A walks behind B (LiDAR + camera occluded)
    python3 tools/test_click_follow.py wall      # A out of view against a wall
    python3 tools/test_click_follow.py hug       # A beside the rover, then inside the LiDAR minimum
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
from dagger_msgs.msg import PersonDetection, PersonDetections  # noqa: E402

# a stable appearance per person (+ per-frame noise), like OSNet signatures
_rng = np.random.default_rng(3)
LOOKS = {}


def looks(name):
    if name not in LOOKS:
        v = _rng.normal(size=512)
        LOOKS[name] = v / np.linalg.norm(v)
    v = LOOKS[name] + _rng.normal(size=512) * 0.03
    return (v / np.linalg.norm(v)).astype(np.float32)

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
        d = min([ray_circle(a, px, py, 0.25) for k_, (px, py) in people.items()
                 if not k_.startswith("SIGN")] +
                [ray_seg(a, *w) for w in walls] + [math.inf])
        rng.append(d if d <= 20.0 else math.inf)
    s.ranges = rng
    det = PersonDetections(); det.header.stamp = stamp; det.header.frame_id = "d435_color_optical_frame"
    det.image_width, det.image_height = W, H
    seen = []
    for name, (px, py) in people.items():
        if name.startswith("SIGN"):
            continue                                  # objects are added below, not raycast
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
        hh = FY * 1.7 / r
        d = PersonDetection()
        d.x1, d.x2 = float(max(0.0, u - half)), float(min(W, u + half))
        d.y1, d.y2 = float(max(0.0, CY - hh / 2)), float(min(H, CY + hh / 2))   # cut by the frame
        d.score = 0.9
        d.embedding = looks(name).tolist()
        det.detections.append(d)
    for name, spec in people.items():                 # a sign: weak, never seen by the LiDAR
        if name.startswith("SIGN"):
            u, score = spec
            d = PersonDetection()
            d.x1, d.x2, d.y1, d.y2, d.score = u - 25.0, u + 25.0, 150.0, 260.0, float(score)
            d.embedding = looks(name).tolist()
            det.detections.append(d)
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
    "reid": ([(9.0, -8.0, 9.0, 8.0)], [
        (0, 3, lambda f: {"A": (3.0, 0.3), "B": (6.5, -2.5)}, "A ahead, B far right; click A, FOLLOW"),
        (3, 5, lambda f: {"A": lerp((3.0, 0.3), (2.0, 3.5), f), "B": (6.5, -2.5)},
         "A walks out of view to the left ..."),
        (5, 12, lambda f: {"B": lerp((6.5, -2.5), (4.0, -0.3), f)},
         "... and disappears (behind a building); B (different looks) walks to the middle"),
        (12, 16, lambda f: {"A": lerp((2.0, 3.5), (3.5, 0.8), f), "B": (4.0, -0.3)},
         "A comes back (a new track) -> must be re-identified"),
        (16, 22, lambda f: {"A": (3.5, 0.8), "B": (4.0, -0.3)}, "A and B both in view")]),
    "beside": ([(9.0, -8.0, 9.0, 8.0)], [
        (0, 3, lambda f: {"A": (3.0, 0.3)}, "A ahead; click A, FOLLOW"),
        (3, 5, lambda f: {"A": lerp((3.0, 0.3), (1.2, 2.0), f)}, "A walks beside the rover (out of view)"),
        (5, 17, lambda f: {"A": (1.2, 2.0)}, "A stands beside the rover 12 s (isolated)"),
        (17, 20, lambda f: {"A": lerp((1.2, 2.0), (3.0, 0.3), f)}, "A walks back in front"),
        (20, 22, lambda f: {"A": (3.0, 0.3)}, "A ahead")]),
    "close": ([(9.0, -8.0, 9.0, 8.0)], [
        (0, 3, lambda f: {"A": (3.0, 0.1)}, "A ahead; click A, FOLLOW"),
        (3, 5, lambda f: {"A": lerp((3.0, 0.1), (0.8, 0.05), f)}, "A walks right up to the rover"),
        (5, 11, lambda f: {"A": (0.8, 0.05)}, "A stands 0.8 m away (inside the LiDAR's 1.0 m minimum)"),
        (11, 13, lambda f: {"A": lerp((0.8, 0.05), (3.0, 0.1), f)}, "A steps back"),
        (13, 15, lambda f: {"A": (3.0, 0.1)}, "A ahead")]),
    # reid2 (2026-09-30): lost 1.5 m away at +45 deg and the search turned toward them
    "hug": ([(9.0, -8.0, 9.0, 8.0)], [
        (0, 3, lambda f: {"A": (3.0, 0.3)}, "A ahead; click A, FOLLOW"),
        (3, 5, lambda f: {"A": lerp((3.0, 0.3), (0.9, 0.9), f)},
         "A walks up BESIDE the rover: 1.27 m at +45 deg (out of view, on the LiDAR)"),
        (5, 9, lambda f: {"A": (0.9, 0.9)}, "A stands there -- inside the 1.03 m corner sweep + margin"),
        (9, 10, lambda f: {"A": lerp((0.9, 0.9), (0.5, 0.75), f)},
         "A steps closer: 0.9 m, inside the LiDAR's 1.0 m minimum -> LOST"),
        (10, 15, lambda f: {"A": (0.5, 0.75)}, "A stays there, invisible to camera and LiDAR"),
        (15, 18, lambda f: {"A": lerp((0.5, 0.75), (3.0, 0.3), f)}, "A walks back out in front"),
        (18, 22, lambda f: {"A": (3.0, 0.3)}, "A ahead")]),
    "sign": ([(9.0, -8.0, 9.0, 8.0)], [
        (0, 3, lambda f: {"A": (3.0, 0.3), "SIGN": (500.0, 0.6)}, "A + a sign that scores 0.6 once"),
        (3, 12, lambda f: {"A": (3.0, 0.3), "SIGN": (500.0, 0.3)}, "the sign keeps scoring 0.3")]),
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
             for exe, args in (("person_tracker", []),
                               ("follow_controller", ["-p", "resume_window:=30.0"]))]
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
    p_det = n.create_publisher(PersonDetections, "/follow/person_detections", 10)
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
    n.create_subscription(PoseStamped, "/follow/search_hint", lambda m: LOG.append(
        ("hint", time.time() - T0[0], m.pose.position.x, m.pose.position.y)), 10)
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

    truth, sel_id, clicked, enabled, sign_clicked = [], None, False, False, False
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
            if scen == "sign" and clicked and not sign_clicked and el > 4.0 and tracks[0] is not None:
                unr = [tr.id for tr in tracks[0].tracks if not tr.ranged]
                if unr:
                    for _ in range(3):
                        p_sel.publish(Int32(data=unr[0]))
                    sign_clicked = True
                    LOG.append(("mark", el, f"operator clicks the SIGN #{unr[0]}"))
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
    def off_a(t, x, y, src_):
        a_ = at(truth, t).get("A")
        if a_ is None:
            return True
        if src_ == "near":                      # 'too close' target: bearing only, range = near_range
            return abs(math.atan2(y, x) - math.atan2(a_[1], a_[0] - 0.2925)) > 0.35   # camera bearing
        return math.hypot(x - a_[0], y - a_[1]) > 0.6
    srcs = sorted(src)
    def src_at(t):
        prev = [s_ for tt, s_ in srcs if tt <= t + 0.02]
        return prev[-1] if prev else "camera"
    far_from_a = [t for t, x, y in tg if off_a(t, x, y, src_at(t))]
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
        # A stands out of view, ISOLATED (B is 3.4 m away): the isolated hold keeps them
        lost = [t for t, n_, m in logs if "LOST" in m]
        res.append(("A standing out of view (isolated) is held, not lost, for the whole scene",
                    not lost and bool(lid) and max(t for t, s_ in src if s_ == "lidar") > 22))
        near_b = [t for t, x, y in tg if t > 12 and math.hypot(x - 4.0, y) < 0.6]
        res.append(("B is never the target", not near_b))
    elif scen == "cross":
        res.append(("A keeps its id through the crossing (targets all A)", not far_from_a))
    elif scen == "occlude":
        near_b = [t for t, x, y in tg if math.hypot(x - 3.0, y) < 0.6]
        res.append(("while A is hidden behind B, B is never the target", not near_b))
        states = {s for _, s in st}
        res.append(("controller never drives toward B",
                    not any(v > 0.01 and near_b for t, v, w in cm), f"states {sorted(states)}"))
    elif scen == "reid":
        srch = [t for t, n_, m in logs if "searching" in m and "LOST" in m]
        reid = [t for t, n_, m in logs if "RE-IDENTIFIED" in m]
        res.append(("A lost -> SEARCHING (re-identification window)", bool(srch)))
        res.append(("A comes back -> RE-IDENTIFIED with the SAME id", bool(reid)))
        back = [q for q in tg if reid and q[0] > reid[0]]
        res.append(("after re-identification the targets are A again", bool(back)))
        s_after = [s_ for t, s_ in st if reid and t > reid[0] + 0.5]
        res.append(("following resumed by itself (no new FOLLOW)", "following" in s_after))
        drv = [v for t, v, w in cm if srch and srch[0] < t < (reid[0] if reid else 1e9) and v > 0.01]
        res.append(("while searching: rotation only, never forward", not drv))
    elif scen == "beside":
        lost = [t for t, n_, m in logs if "LOST" in m]
        res.append(("standing 12 s beside the rover (isolated): never lost", not lost))
        gaps = [b_[0] - a_[0] for a_, b_ in zip(tg, tg[1:]) if 5 < a_[0] < 17]
        res.append(("held on the LiDAR the whole time (no gap > 1 s)", gaps and max(gaps) < 1.0,
                    f"max gap {max(gaps):.2f} s" if gaps else "no targets"))
        res.append(("controller never disabled", not any(s_.startswith("disabled: target") for t, s_ in st)))
    elif scen == "close":
        near = [t for t, s_ in src if s_ == "near"]
        res.append(("0.8 m away (below LiDAR minimum): still the target (source 'near')", bool(near)))
        res.append(("never lost", not any("LOST" in m for t, n_, m in logs)))
        drv = [v for t, v, w in cm if v > 0.01 and math.hypot(*at(truth, t)["A"]) < 2.6]
        res.append(("never drives forward while A is closer than 2.6 m", not drv, f"{len(drv)} cmds"))

        turn = [t for t, v, w in cm if abs(w) > 1e-3 and math.hypot(*at(truth, t)["A"]) < 1.4
                and math.hypot(*at(truth, t - 0.3)["A"]) < 1.4]
        res.append(("never turns in place while A is within reach (< 1.4 m)", not turn,
                    f"{len(turn)} cmds"))
    elif scen == "hug":
        rng_a = lambda t: math.hypot(*at(truth, t)["A"])  # noqa: E731
        # 0.3 s of pipeline lag either side of the 1.6 m line
        turn = [t for t, v, w in cm if abs(w) > 1e-3 and rng_a(t) < 1.4 and rng_a(t - 0.3) < 1.4]
        hints = [(e[1], math.hypot(e[2], e[3])) for e in LOG if e[0] == "hint"]
        h_at = lambda t: [r for tt, r in hints if tt <= t][-1:]  # noqa: E731
        res.append(("never turns in place while A is within reach (< 1.4 m)", not turn,
                    f"{len(turn)} turning cmds, first at {turn[0]:.1f} s (hint range then "
                    f"{h_at(turn[0])}, last hint at {hints[-1][0] if hints else None})" if turn else ""))
        drv = [t for t, v, w in cm if v > 0.01 and rng_a(t) < 2.6]
        res.append(("never drives forward while A is closer than 2.6 m", not drv, f"{len(drv)} cmds"))
        lost = [t for t, n_, m in logs if "LOST" in m]
        res.append(("inside the LiDAR minimum, out of view -> lost (searching, not a guess)", bool(lost)))
        back = [t for t, n_, m in logs if "RE-IDENTIFIED" in m or "reacquired" in m]
        s_after = [s_ for t, s_ in st if t > 17.0]
        res.append(("A walks back out in front -> followed again without a new FOLLOW",
                    "following" in s_after, f"re-id/reacquire logs {len(back)}"))
    elif scen == "sign":
        ign = [m for t, n_, m in logs if "ignored" in m]
        res.append(("clicking the sign (never LiDAR-ranged) is refused", bool(ign)))
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
