#!/usr/bin/env python3
"""Offline scene test for follow-me: the REAL target_selector + follow_controller
against a simulated world. NO ROVER, NO FCU.

    python3 tools/test_follow_lidar.py main   # walk out of view L/R, bystander, expiry
    python3 tools/test_follow_lidar.py wall   # person against a wall out of view

Runs in ROS_DOMAIN_ID 42, set below before rclpy is imported and inherited by the
nodes it starts, so it cannot reach a live stack on domain 0. That matters: a
second follow_controller on the live domain WILL drive an armed rover.

The world raycasts a 360 deg scan from people (0.25 m circles) and walls, and
produces camera detections only for people inside the 55.7 deg field of view.
Results go to /tmp/follow_sim_<scenario>.json; the report prints per second.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "42"
import math, os, signal, subprocess, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Bool, Int32, String
from rcl_interfaces.msg import Log
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose
import tf2_ros

SCEN = sys.argv[1] if len(sys.argv) > 1 else "main"
FX, FY, CX, CY, W, H = 606.04, 605.56, 329.92, 259.31, 640, 480
CAM = np.array([0.2925, 0.0, 0.99])
R = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])   # base <- optical
HALF_FOV = math.atan(W / 2 / FX)

def q_from_R(m):
    t = m.trace(); s = math.sqrt(max(1e-9, t + 1)) * 2
    if t > 0: return ((m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s, (m[1,0]-m[0,1])/s, 0.25*s)
    s = math.sqrt(1 + m[0,0] - m[1,1] - m[2,2]) * 2
    return (0.25*s, (m[0,1]+m[1,0])/s, (m[0,2]+m[2,0])/s, (m[2,1]-m[1,2])/s)

SP = "/tmp"
SRC = "export ROS_DOMAIN_ID=42 && source /opt/ros/humble/setup.bash && source /home/jetson/ros2_ws/install/setup.bash"
procs = [subprocess.Popen(["bash", "-lc", f"{SRC} && exec ros2 run dagger_bringup {n}"],
                          stdout=open(f"{SP}/follow_sim_{n}.log", "w"), stderr=subprocess.STDOUT,
                          preexec_fn=os.setsid) for n in ("target_selector", "follow_controller")]

rclpy.init(); n = Node("scene")
stb = tf2_ros.StaticTransformBroadcaster(n)
def tfm(par, ch, xyz, q):
    t = TransformStamped(); t.header.frame_id = par; t.child_frame_id = ch
    t.header.stamp = n.get_clock().now().to_msg()
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
    return t
stb.sendTransform([tfm("odom", "base_footprint", (0., 0., 0.), (0., 0., 0., 1.)),
                   tfm("base_footprint", "d435_color_optical_frame", tuple(CAM), q_from_R(R))])
be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
p_info = n.create_publisher(CameraInfo, "/d435/color/camera_info", be)
p_scan = n.create_publisher(LaserScan, "/scan_safety", be)
p_det = n.create_publisher(Detection2DArray, "/follow/detections", 10)
p_en = n.create_publisher(Bool, "/follow/enable", 10)
LOG = []
n.create_subscription(PoseStamped, "/follow/target", lambda m: LOG.append(("tgt", time.time(), m.pose.position.x, m.pose.position.y)), be)
n.create_subscription(String, "/follow/target_source", lambda m: LOG.append(("src", time.time(), m.data)), 10)
n.create_subscription(Int32, "/follow/track_id", lambda m: LOG.append(("id", time.time(), m.data)), 10)
n.create_subscription(Twist, "/cmd_vel_follow_raw", lambda m: LOG.append(("cmd", time.time(), m.linear.x, m.angular.z)), 10)
n.create_subscription(Log, "/rosout", lambda m: LOG.append(("log", time.time(), m.name, m.msg)) if m.name in ("target_selector", "follow_controller") and m.level >= 20 else None, 50)

info = CameraInfo(); info.width, info.height = W, H; info.k = [FX, 0., CX, 0., FY, CY, 0., 0., 1.]

def ray_circle(a, cx, cy, r):
    dx, dy = math.cos(a), math.sin(a); b = dx * cx + dy * cy
    c = cx * cx + cy * cy - r * r; disc = b * b - c
    if disc < 0: return math.inf
    t = b - math.sqrt(disc); return t if t > 0 else math.inf
def ray_seg(a, x1, y1, x2, y2):
    dx, dy = math.cos(a), math.sin(a); ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-9: return math.inf
    t = (x1 * ey - y1 * ex) / den; u = (x1 * dy - y1 * dx) / den
    return t if t > 0 and 0 <= u <= 1 else math.inf

def world(people, walls, stamp):
    s = LaserScan(); s.header.stamp = stamp; s.header.frame_id = "base_footprint"
    s.angle_min, s.angle_max, s.angle_increment = -math.pi, math.pi, 0.0087
    s.range_min, s.range_max = 1.0, 20.0
    nb = int(2 * math.pi / 0.0087); rng = []
    for i in range(nb):
        a = -math.pi + i * 0.0087
        d = min([ray_circle(a, px, py, 0.25) for px, py in people.values()] +
                [ray_seg(a, *w) for w in walls] + [math.inf])
        rng.append(d if d <= 20.0 else math.inf)
    s.ranges = rng
    det = Detection2DArray(); det.header.stamp = stamp; det.header.frame_id = "d435_color_optical_frame"
    for name, (px, py) in people.items():
        v = R.T @ (np.array([px, py, 1.0]) - CAM)
        if v[2] <= 0.3: continue
        u = FX * v[0] / v[2] + CX
        if not (15 <= u <= W - 15): continue            # outside the field of view
        r = math.hypot(px - CAM[0], py - CAM[1])
        d = Detection2D(); d.header = det.header; d.bbox = BoundingBox2D()
        d.bbox.center.position.x, d.bbox.center.position.y = float(u), float(FY * v[1] / v[2] + CY)
        d.bbox.size_x, d.bbox.size_y = float(2 * FX * math.atan2(0.25, r)), 300.0
        h = ObjectHypothesisWithPose(); h.hypothesis.class_id = "0"; h.hypothesis.score = 0.9
        d.results.append(h); det.detections.append(d)
    return s, det

def report(path):
    import json
    d = json.load(open(path)); T0 = d["T0"]; L = d["log"]
    print("--- node log ---")
    for e in L:
        if e[0] == "log": print(f"  {e[1]-T0:6.2f}s  {e[2]}: {e[3][:95]}")
    print("\n--- per second: source / track / target position / command ---")
    for sec in range(int(max(e[1] for e in L) - T0) + 1):
        w = [e for e in L if sec <= e[1] - T0 < sec + 1]
        src = sorted({e[2] for e in w if e[0] == "src"}); ids = sorted({e[2] for e in w if e[0] == "id"})
        tg = [e for e in w if e[0] == "tgt"]; cm = [e for e in w if e[0] == "cmd"]
        note = next((m[1] for m in d["marks"] if m[0] == sec), "")
        t = f"({tg[-1][2]:+.2f},{tg[-1][3]:+.2f}) brg {math.degrees(math.atan2(tg[-1][3], tg[-1][2])):+5.1f}" if tg else "        -              "
        c = f"v {cm[-1][2]:+.2f} w {cm[-1][3]:+.2f}" if cm else "  (silent)  "
        print(f"  {sec:2d}s  {','.join(src) or '-':12s} id {ids or '-'}  tgt {t}  {c}   {note}")


def check(path, scen):
    import json
    d = json.load(open(path)); T0 = d["T0"]; L = d["log"]
    tg = [(e[1] - T0, e[2], e[3]) for e in L if e[0] == "tgt"]
    cm = [(e[1] - T0, e[2], e[3]) for e in L if e[0] == "cmd"]
    src = [(e[1] - T0, e[2]) for e in L if e[0] == "src"]
    ids = [(e[1] - T0, e[2]) for e in L if e[0] == "id"]
    logs = [(e[1] - T0, e[2], e[3]) for e in L if e[0] == "log"]
    brg = lambda x, y: abs(math.degrees(math.atan2(y, x)))
    res = []
    if scen == "main":
        lid = [t for t, sname in src if sname == "lidar"]
        # Any target beyond the camera's 27.8 deg half-FOV can only have come
        # from the LiDAR, so reaching 45 deg proves the hold carried it there.
        res.append(("LiDAR holds the target out of camera view",
                    bool(lid) and max(brg(x, y) for t, x, y in tg) > 45))
        # every command issued while the latest target was > 35 deg off must not drive
        bad = 0
        for t, v, w in cm:
            prev = [q for q in tg if q[0] <= t]
            if prev and brg(prev[-1][1], prev[-1][2]) > 36 and v > 0.01:
                bad += 1
        res.append(("turns in place (v = 0) beyond 35 deg bearing", bad == 0))
        sw = [t for t, n_, m in logs if "TARGET SWITCHED" in m]
        res.append(("no hop to the bystander while the track is held (id stays 1 until expiry)",
                    all(i == 1 for t, i in ids if t < (sw[0] if sw else 1e9)) and bool(sw) and sw[0] > 14.0))
        dis = [t for t, n_, m in logs if "TARGET CHANGED" in m]
        after = [v for t, v, w in cm if dis and t > dis[0] + 0.2 and abs(v) > 0.01]
        res.append(("controller disables on the switch and never drives at the bystander",
                    bool(dis) and not after))
    else:
        refused = [t for t, n_, m in logs if "not a person, refusing" in m]
        res.append(("refuses to follow a wall-merged cluster", bool(refused)))
        late = [t for t, x, y in tg if refused and t > refused[0] + 0.5]
        res.append(("publishes no target once refused", not late))
        res.append(("never drives forward during the whole scene",
                    all(v <= 0.01 for t, v, w in cm)))
    print()
    for name, ok in res:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    print(f"\n{sum(ok for _, ok in res)}/{len(res)} passed")
    return all(ok for _, ok in res)


def lerp(a, b, f): return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)

# (t0, t1, people_fn(f) -> dict, walls, note)
if SCEN == "main":
    WALLS = [(7.0, -8.0, 7.0, 8.0)]
    PH = [(0, 3,  lambda f: {"A": (3.0, 0.3)}, "A ahead, in view"),
          (3, 7,  lambda f: {"A": lerp((3.0, 0.3), (1.8, 2.8), f)}, "A walks out LEFT of the FOV"),
          (7, 9,  lambda f: {"A": lerp((1.8, 2.8), (3.0, 0.5), f)}, "A back into view"),
          (9, 12, lambda f: {"A": lerp((3.0, 0.5), (2.0, -2.8), f), "B": (4.0, 0.0)}, "B appears; A walks out RIGHT"),
          (12, 21, lambda f: {"A": (2.0, -2.8), "B": (4.0, 0.0)}, "A stays out of view; B in view")]
else:   # wall: A walks beside a wall, out of view
    WALLS = [(7.0, -8.0, 7.0, 8.0), (0.5, 2.3, 6.0, 2.3)]
    PH = [(0, 3, lambda f: {"A": (3.0, 0.3)}, "A ahead"),
          (3, 7, lambda f: {"A": lerp((3.0, 0.3), (2.0, 1.95), f)}, "A walks out LEFT, against a wall"),
          (7, 12, lambda f: {"A": (2.0, 1.95)}, "A stands against the wall, out of view")]

time.sleep(8)
T0 = time.time(); marks = []
en_sent = False
for t0, t1, fn, note in PH:
    marks.append((t0, note))
    while time.time() - T0 < t1:
        el = time.time() - T0
        if not en_sent and el > 1.0:
            for _ in range(5): p_en.publish(Bool(data=True))
            en_sent = True
        st = n.get_clock().now().to_msg()
        scan, det = world(fn((el - t0) / (t1 - t0)), WALLS, st)
        p_info.publish(info); p_scan.publish(scan); p_det.publish(det)
        end = time.time() + 0.1
        while time.time() < end: rclpy.spin_once(n, timeout_sec=0.005)

import json
json.dump({"T0": T0, "marks": marks, "log": LOG}, open(f"{SP}/follow_sim_{SCEN}.json", "w"))
for p in procs:
    try: os.killpg(os.getpgid(p.pid), signal.SIGINT)
    except ProcessLookupError: pass
time.sleep(3)
for p in procs:
    try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except ProcessLookupError: pass
print("done", len(LOG), "events\n")
report(f"{SP}/follow_sim_{SCEN}.json")
sys.exit(0 if check(f"{SP}/follow_sim_{SCEN}.json", SCEN) else 1)
