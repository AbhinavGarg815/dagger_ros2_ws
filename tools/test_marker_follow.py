#!/usr/bin/env python3
"""Scene test for ArUco-marker following: the REAL marker_tracker + person_tracker +
follow_controller in a simulated world, with a scripted operator. NO ROVER, NO FCU.

    python3 tools/test_marker_follow.py follow   # click marker 7, FOLLOW; it turns away 4 s; comes back
    python3 tools/test_marker_follow.py switch   # marker -> person -> marker: one target source at a time
    python3 tools/test_marker_follow.py all

Runs in ROS_DOMAIN_ID 42 (test_click_follow sets it before rclpy is imported), so
it cannot reach a live stack on domain 0.

World (test_click_follow's): the rover stays at the odom origin; people are 0.25 m
circles. A CARRIER (car, rover) is a 0.6 m wide rear face the LiDAR sees, with a
15 cm marker on it at camera height, facing the way given; the marker's corners
are projected through the sim camera exactly as person_detector would report
them. A carrier whose marker is 'hidden' keeps its LiDAR return (it turned away).
"""
import math
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
import test_click_follow as tc  # noqa: E402  (sets ROS_DOMAIN_ID=42, imports rclpy)

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import tf2_ros  # noqa: E402
from dagger_msgs.msg import MarkerDetection, MarkerDetections, MarkerTracks, PersonTracks  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo  # noqa: E402
from std_msgs.msg import Bool, Int32, String  # noqa: E402

BASE = 1_000_000
SIZE = 0.15
lerp = tc.lerp


def marker_corners(x, y, yaw_deg=180.0, z=1.0):
    """Corners (8 floats, ArUco order) of a marker centred at base (x, y, z) whose
    printed face points along yaw_deg (180 = at the rover), or None if not fully in view."""
    n = np.array([math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg)), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(-n, up)
    c, h = np.array([x, y, z]), SIZE / 2
    out = []
    for p in (c - right * h + up * h, c + right * h + up * h, c + right * h - up * h, c - right * h - up * h):
        v = tc.R.T @ (p - tc.CAM)
        if v[2] <= 0.3:
            return None
        u, w = tc.FX * v[0] / v[2] + tc.CX, tc.FY * v[1] / v[2] + tc.CY
        if not (2 <= u <= tc.W - 2 and 2 <= w <= tc.H - 2):
            return None
        out += [float(u), float(w)]
    return out


# carriers: name -> (marker id, (x, y), marker visible?)   people: name -> (x, y)
SCENES = {
    "follow": [
        (0, 3, lambda f: ({"C7": (7, (5.0, 0.3), True), "C3": (3, (6.0, -2.0), True)}, {}),
         "marker 7 on a carrier 5 m ahead, marker 3 on another; click 7, FOLLOW"),
        (3, 8, lambda f: ({"C7": (7, lerp((5.0, 0.3), (4.5, 1.6), f), True), "C3": (3, (6.0, -2.0), True)}, {}),
         "carrier 7 drifts left"),
        (8, 12, lambda f: ({"C7": (7, lerp((4.5, 1.6), (4.0, 2.4), f), False), "C3": (3, (6.0, -2.0), True)}, {}),
         "carrier 7 turns away: its marker is not visible for 4 s (marker 3 still is)"),
        (12, 17, lambda f: ({"C7": (7, lerp((4.0, 2.4), (5.0, 0.8), f), True), "C3": (3, (6.0, -2.0), True)}, {}),
         "marker 7 visible again"),
        (17, 19, lambda f: ({"C7": (7, (5.0, 0.8), True), "C3": (3, (6.0, -2.0), True)}, {}), "steady"),
        (19, 23, lambda f: ({"C7": (7, (5.0, 0.8), True), "C3": (3, (6.0, -2.0), True)}, {}),
         "operator presses CLEAR while following (live aruco2: the rover then searched for 29 s)")],
    "switch": [
        (0, 3, lambda f: ({"C7": (7, (5.0, -0.8), True)}, {"A": (3.5, 1.0)}),
         "marker 7 ahead-right, person A ahead-left; click marker 7, FOLLOW"),
        (3, 6, lambda f: ({"C7": (7, (5.0, -0.8), True)}, {"A": (3.5, 1.0)}), "following the marker"),
        (6, 10, lambda f: ({"C7": (7, (5.0, -0.8), True)}, {"A": (3.5, 1.0)}),
         "operator clicks PERSON A (no new FOLLOW)"),
        (10, 14, lambda f: ({"C7": (7, (5.0, -0.8), True)}, {"A": (3.5, 1.0)}),
         "operator clicks MARKER 7 again, presses FOLLOW")],
}


def run(scen):
    phases = SCENES[scen]
    procs = [subprocess.Popen([f"{tc.LIB}/{exe}", "--ros-args", *args], start_new_session=True,
                              stdout=open(f"/tmp/marker_sim_{scen}_{exe}.log", "w"),
                              stderr=subprocess.STDOUT)
             for exe, args in (("person_tracker", []),
                               ("marker_tracker", ["-p", f"marker_size:={SIZE}", "-p", "marker_ids:=7,3"]),
                               ("follow_controller", ["-p", "resume_window:=30.0"]))]
    n = Node(f"marker_scene_{scen}")
    stb = tf2_ros.StaticTransformBroadcaster(n)
    t = TransformStamped(); t.header.frame_id, t.child_frame_id = "base_footprint", "d435_color_optical_frame"
    t.header.stamp = n.get_clock().now().to_msg()
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = tc.CAM
    (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
     t.transform.rotation.w) = tc.q_from_R(tc.R)
    stb.sendTransform([t])
    be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
    p_info = n.create_publisher(CameraInfo, "/d435/color/camera_info", be)
    p_scan = n.create_publisher(tc.LaserScan, "/scan_safety", be)
    p_det = n.create_publisher(tc.PersonDetections, "/follow/person_detections", 10)
    p_mk = n.create_publisher(MarkerDetections, "/follow/marker_detections", 10)
    p_odom = n.create_publisher(Odometry, "/odometry/filtered", be)
    p_sel = n.create_publisher(Int32, "/follow/select", 10)
    p_en = n.create_publisher(Bool, "/follow/enable", 10)
    LOG, mtracks, ptracks = [], [None], [None]
    T0 = [time.time()]
    el = lambda: time.time() - T0[0]  # noqa: E731
    n.create_subscription(PoseStamped, "/follow/target", lambda m: LOG.append(
        ("tgt", el(), m.pose.position.x, m.pose.position.y)), be)
    n.create_subscription(String, "/follow/target_source", lambda m: LOG.append(("src", el(), m.data)), 10)
    n.create_subscription(Int32, "/follow/track_id", lambda m: LOG.append(("id", el(), m.data)), 10)
    n.create_subscription(String, "/follow/state", lambda m: LOG.append(("state", el(), m.data)), 10)
    n.create_subscription(Twist, "/cmd_vel_follow_raw", lambda m: LOG.append(
        ("cmd", el(), m.linear.x, m.angular.z)), 10)
    n.create_subscription(MarkerTracks, "/follow/marker_tracks", lambda m: (
        mtracks.__setitem__(0, m), LOG.append(("mstate", el(), m.selected_id, m.target_state,
                                               sorted(k.marker_id for k in m.tracks if k.confirmed)))), 10)
    n.create_subscription(PersonTracks, "/follow/tracks", lambda m: (
        ptracks.__setitem__(0, m), LOG.append(("pstate", el(), m.selected_id, m.target_state))), 10)
    n.create_subscription(Log, "/rosout", lambda m: LOG.append(("log", el(), m.name, m.msg))
                          if m.name in ("person_tracker", "marker_tracker", "follow_controller")
                          and m.level >= 20 else None, 50)
    info = CameraInfo(); info.width, info.height = tc.W, tc.H
    info.k = [tc.FX, 0., tc.CX, 0., tc.FY, tc.CY, 0., 0., 1.]
    info.d = [0.0] * 5

    def odom():
        o = Odometry(); o.header.frame_id, o.child_frame_id = "odom", "base_footprint"
        o.header.stamp = n.get_clock().now().to_msg(); o.pose.pose.orientation.w = 1.0
        p_odom.publish(o)

    n.create_timer(0.05, odom)
    end = time.time() + 6.0                            # nodes up + discovery
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

    def select(tid, note):
        for _ in range(3):
            p_sel.publish(Int32(data=int(tid)))
        LOG.append(("mark", el(), note))

    def follow():
        for _ in range(3):
            p_en.publish(Bool(data=True))
        LOG.append(("mark", el(), "operator presses FOLLOW"))

    truth, done = [], set()
    LOG.clear()                                        # warm-up entries are on the old clock
    T0[0] = time.time()
    for t0, t1, fn, note in phases:
        LOG.append(("mark", t0, note))
        while el() < t1:
            e = el()
            carriers, people = fn((e - t0) / (t1 - t0))
            truth.append((e, carriers, people))
            walls = [(x, y - 0.3, x, y + 0.3) for _, (x, y), _ in carriers.values()]   # rear faces
            scan, det = tc.world(n, people, walls)
            mk = MarkerDetections(); mk.header = det.header
            mk.image_width, mk.image_height, mk.dictionary = tc.W, tc.H, "DICT_4X4_50"
            for mid, (x, y), vis in carriers.values():
                c = marker_corners(x - 0.01, y) if vis else None
                if c is not None:
                    d = MarkerDetection(); d.marker_id, d.corners = mid, c
                    mk.markers.append(d)
            p_info.publish(info); p_scan.publish(scan); p_det.publish(det); p_mk.publish(mk)
            m7 = mtracks[0] is not None and any(k.marker_id == 7 and k.confirmed for k in mtracks[0].tracks)
            if "click7" not in done and e > 1.0 and m7:
                select(BASE + 7, "operator clicks marker 7"); done.add("click7")
            if "follow" not in done and "click7" in done and e > 1.6:
                follow(); done.add("follow")
            if scen == "follow" and "clear" not in done and e > 19.0:
                select(-1, "operator presses CLEAR"); done.add("clear")
            if scen == "switch":
                if "clickA" not in done and e > 6.0 and ptracks[0] is not None and ptracks[0].tracks:
                    select(ptracks[0].tracks[0].id, f"operator clicks person #{ptracks[0].tracks[0].id}")
                    done.add("clickA")
                if "click7b" not in done and e > 10.0:
                    select(BASE + 7, "operator clicks marker 7 again"); done.add("click7b")
                if "follow2" not in done and e > 10.6:
                    follow(); done.add("follow2")
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
    return LOG, truth


def evaluate(scen, LOG, truth):
    tg = [(e[1], e[2], e[3]) for e in LOG if e[0] == "tgt"]
    cm = [(e[1], e[2], e[3]) for e in LOG if e[0] == "cmd"]
    src = [(e[1], e[2]) for e in LOG if e[0] == "src"]
    ids = [(e[1], e[2]) for e in LOG if e[0] == "id"]
    st = [(e[1], e[2]) for e in LOG if e[0] == "state"]
    ms = [(e[1], e[2], e[3], e[4]) for e in LOG if e[0] == "mstate"]
    ps = [(e[1], e[2], e[3]) for e in LOG if e[0] == "pstate"]
    logs = [(e[1], e[2], e[3]) for e in LOG if e[0] == "log"]
    marks = {e[2]: e[1] for e in LOG if e[0] == "mark"}

    def at(t):
        return min(truth, key=lambda e: abs(e[0] - t))

    def d_to(t, x, y, name):
        c = at(t)[1].get(name)
        return math.hypot(x - c[1][0], y - c[1][1]) if c else math.inf

    res = [("marker 7 confirmed and selected by the click", any(s == BASE + 7 for _, s, _, _ in ms))]
    if scen == "follow":
        off = [t for t, x, y in tg if d_to(t, x, y, "C7") > 0.35]
        res.append(("every target is marker 7's position (within 0.35 m)", bool(tg) and not off,
                    f"{len(off)} of {len(tg)} off"))
        res.append(("marker 3 (in view the whole time) is never the target",
                    not [t for t, x, y in tg if d_to(t, x, y, "C3") < 1.0]))
        res.append(("/follow/track_id is always 1000007", bool(ids) and all(i == BASE + 7 for _, i in ids)))
        res.append(("range from the LiDAR where it agrees (source marker+lidar)",
                    any(s == "marker+lidar" for _, s in src)))
        drive = [v for t, v, w in cm if 2.0 < t < 8.0]
        res.append(("drives toward the marker while it is > 3.25 m away", bool(drive) and max(drive) > 0.1))
        gap = [t for t, x, y in tg if 8.7 < t < 12.0]
        res.append(("marker not visible: no target (nothing is guessed)", not gap, f"{len(gap)} targets"))
        res.append(("... tracker says SEARCHING", any(s == "SEARCHING" for t, _, s, _ in ms if 8.7 < t < 12.0)))
        srch = [(t, v, w) for t, v, w in cm if 9.3 < t < 12.0]
        res.append(("... controller searches: rotation only, never forward",
                    bool(srch) and all(v <= 0.01 for _, v, _ in srch) and any(abs(w) > 0.1 for _, _, w in srch)))
        again = [t for t, n_, m in logs if "SEEN AGAIN" in m]
        res.append(("marker 7 back in view -> 'SEEN AGAIN', same id", bool(again)))
        res.append(("following resumes by itself (no new FOLLOW)",
                    "following" in [s for t, s in st if again and t > again[0] + 0.5]))
        tc_ = marks.get("operator presses CLEAR", 1e9)
        tf_ = marks.get("operator presses FOLLOW", 1e9)
        res.append(("controller never disabled between FOLLOW and CLEAR",
                    tf_ < tc_ and not any(s.startswith("disabled") for t, s in st if tf_ + 0.5 < t < tc_)))
        res.append(("CLEAR while following -> controller disabled: selection cleared, within 0.3 s",
                    any(s == "disabled: selection cleared" for t, s in st if tc_ <= t < tc_ + 0.3)))
        after = [(t, v, w) for t, v, w in cm if t > tc_ + 0.3 and (abs(v) > 0.01 or abs(w) > 0.01)]
        res.append(("... and no motion is commanded afterwards (no search for a cleared target)",
                    not after, f"{len(after)} cmds"))
    else:
        tA = marks.get("operator clicks marker 7 again", 1e9)
        tA0 = next((t for k, t in marks.items() if k.startswith("operator clicks person")), 1e9)
        first = [(t, x, y) for t, x, y in tg if t < tA0]
        res.append(("first the marker is the target", bool(first) and all(d_to(t, x, y, "C7") < 0.35 for t, x, y in first)))
        mid_m = [s for t, s, _, _ in ms if tA0 + 0.5 < t < tA]
        res.append(("person clicked -> the marker selection is cleared", bool(mid_m) and all(s == -1 for s in mid_m)))
        mid_p = [s for t, s, _ in ps if tA0 + 0.5 < t < tA]
        res.append(("... and the person is selected", bool(mid_p) and all(s >= 0 for s in mid_p)))
        dis = [t for t, n_, m in logs if "TARGET CHANGED" in m]
        res.append(("the controller DISABLES on the change of target", bool(dis) and tA0 < dis[0] < tA0 + 1.5))
        moved = [v for t, v, w in cm if dis and dis[0] + 0.3 < t < tA and (abs(v) > 0.01 or abs(w) > 0.01)]
        res.append(("... and commands nothing toward the person", not moved, f"{len(moved)} cmds"))
        mid_t = [(t, x, y) for t, x, y in tg if tA0 + 0.6 < t < tA]
        res.append(("while the person is selected, targets are the person's only",
                    bool(mid_t) and all(math.hypot(x - 3.5, y - 1.0) < 0.6 for _, x, y in mid_t)))
        last_p = [s for t, s, _ in ps if t > tA + 0.5]
        res.append(("marker clicked again -> the person selection is cleared",
                    bool(last_p) and all(s == -1 for s in last_p)))
        last_t = [(t, x, y) for t, x, y in tg if t > tA + 0.6]
        res.append(("... and targets are the marker's only",
                    bool(last_t) and all(d_to(t, x, y, "C7") < 0.35 for t, x, y in last_t)))
        seq = []
        for _, i in ids:
            if not seq or seq[-1] != i:
                seq.append(i)
        res.append(("/follow/track_id: marker, person, marker -- never interleaved",
                    len(seq) == 3 and seq[0] == seq[2] == BASE + 7 and 0 <= seq[1] < BASE, str(seq)))
        res.append(("following the marker again needed a new FOLLOW",
                    "following" in [s for t, s in st if t > tA + 1.2]))
    print(f"\n=== {scen} ===")
    for e in LOG:
        if e[0] == "mark":
            print(f"  {e[1]:6.2f}s  -- {e[2]}")
        elif e[0] == "log":
            print(f"  {e[1]:6.2f}s  {e[2]}: {e[3][:110]}")
    for item in res:
        name, ok = item[0], item[1]
        detail = item[2] if len(item) > 2 else ""
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail and not ok else ""))
    return [bool(ok) for _, ok, *_ in res]


def main():
    scen = sys.argv[1] if len(sys.argv) > 1 else "follow"
    rclpy.init()
    allres = []
    for s in (SCENES if scen == "all" else [scen]):
        allres += evaluate(s, *run(s))
    rclpy.try_shutdown()
    print(f"\n{sum(allres)}/{len(allres)} passed", flush=True)
    os._exit(0 if allres and all(allres) else 1)


if __name__ == "__main__":
    main()
