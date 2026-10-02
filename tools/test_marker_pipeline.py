#!/usr/bin/env python3
"""End-to-end test of the marker path with the REAL nodes. NO ROVER, NO FCU, no GPU.

    python3 tools/test_marker_pipeline.py
    python3 tools/test_marker_pipeline.py --people   # YOLOX-S + re-ID loaded too (needs the GPU:
                                                     # T4 must be down), as follow.launch runs it

    rendered camera frames (tools/marker_sim.py: the rover's real intrinsics and
    upside-down mount)  ->  person_detector (detect_people:=false, ArUco on)
    ->  /follow/marker_detections  ->  marker_tracker  ->  /follow/marker_tracks,
    /follow/target

Runs in ROS_DOMAIN_ID 42 (set before rclpy is imported, inherited by the nodes it
starts), so it cannot reach a live stack. Checks what the unit and scene tests
cannot: the image really goes through person_detector (rgb8 in, native
orientation), the camera mount is read from /tf_static, and the range / bearing
that come out are the marker's.
"""
import array
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
from dagger_msgs.msg import MarkerDetections, MarkerTracks, PersonDetections  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo, CompressedImage, Image  # noqa: E402
from std_msgs.msg import Int32  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
from marker_sim import CAM_Q, CAM_T, H, K, W, render  # noqa: E402

LIB = "/home/jetson/ros2_ws/install/dagger_bringup/lib/dagger_bringup"
SIZE = 0.15
results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""), flush=True)


PEOPLE = "--people" in sys.argv
M = "/home/jetson/ros2_ws/models"
DET = (["-p", f"engine_path:={M}/yolox_s_fp16.engine", "-p", f"reid_engine:={M}/osnet_x0_25_msmt17_fp16.engine",
        "-p", "conf_threshold:=0.2"] if PEOPLE else ["-p", "detect_people:=false"])
procs = [subprocess.Popen([f"{LIB}/{exe}", "--ros-args", *args], start_new_session=True,
                          stdout=open(f"/tmp/marker_pipe_{exe}.log", "w"), stderr=subprocess.STDOUT)
         for exe, args in (("person_detector", [*DET,
                                                "-p", "marker_dictionary:=DICT_4X4_50",
                                                "-p", "image_rotation_deg:=180"]),
                           ("marker_tracker", ["-p", f"marker_size:={SIZE}", "-p", "marker_ids:=7",
                                               "-p", "image_rotation_deg:=180"]))]
rclpy.init()
n = Node("marker_pipe_test")
be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
stb = tf2_ros.StaticTransformBroadcaster(n)
t = TransformStamped()
t.header.frame_id, t.child_frame_id = "base_footprint", "d435_color_optical_frame"
t.header.stamp = n.get_clock().now().to_msg()
t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in CAM_T)
(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = CAM_Q
stb.sendTransform([t])
p_img = n.create_publisher(Image, "/d435/color/image_raw", be)
p_info = n.create_publisher(CameraInfo, "/d435/color/camera_info", be)
p_odom = n.create_publisher(Odometry, "/odometry/filtered", be)
p_sel = n.create_publisher(Int32, "/follow/select", 10)
dets, tracks, targets, people = [], [], [], []
n.create_subscription(MarkerDetections, "/follow/marker_detections", dets.append, 10)
n.create_subscription(MarkerTracks, "/follow/marker_tracks", tracks.append, 10)
arrive = {}                                         # stamp -> {"ui": t, "people": t}: who is out first


def seen(kind):
    def f(m):
        arrive.setdefault((m.header.stamp.sec, m.header.stamp.nanosec), {})[kind] = time.monotonic()
        if kind == "people":
            people.append(m)
    return f


n.create_subscription(PersonDetections, "/follow/person_detections", seen("people"), 10)
n.create_subscription(CompressedImage, "/follow/ui/image/compressed", seen("ui"),
                      QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5))
n.create_subscription(PoseStamped, "/follow/target", lambda m: targets.append(
    (m.pose.position.x, m.pose.position.y)), 10)
info = CameraInfo()
info.width, info.height = W, H
info.k = [K[0], 0.0, K[2], 0.0, K[1], K[3], 0.0, 0.0, 1.0]
info.d = [0.0] * 5
info.header.frame_id = "d435_color_optical_frame"


def image(frame_bgr):
    m = Image()
    m.header.frame_id = "d435_color_optical_frame"
    m.height, m.width, m.encoding, m.step = H, W, "rgb8", 3 * W      # the driver streams rgb8
    # array('B'), built once per scene: rclpy validates a bytes value per element
    # (~180 ms for this 921 kB image), which throttled the whole test to 3 Hz
    m.data = array.array("B", frame_bgr[:, :, ::-1].tobytes())
    return m


def publish(m):
    m.header.stamp = n.get_clock().now().to_msg()
    info.header.stamp = m.header.stamp
    o = Odometry()
    o.header.stamp, o.header.frame_id, o.child_frame_id = m.header.stamp, "odom", "base_footprint"
    o.pose.pose.orientation.w = 1.0
    p_info.publish(info)
    p_odom.publish(o)
    p_img.publish(m)


def run(frame, secs):
    end = time.time() + secs
    while time.time() < end:
        publish(frame)
        stop = time.time() + 0.1
        while time.time() < stop:
            rclpy.spin_once(n, timeout_sec=0.01)


try:
    X, Y = 3.0, 0.6                                     # marker 7, facing the rover; marker 9 is not ours
    scene = image(render([(7, SIZE, (X, Y, 1.0), 180.0), (9, SIZE, (4.0, -1.0, 1.1), 180.0)]))
    empty = image(render([]))
    run(empty, 30.0 if PEOPLE else 6.0)                 # nodes up (+ TensorRT engines) + discovery
    check("no marker in view: detections are published every frame, empty",
          len(dets) >= 10 and all(not d.markers for d in dets), f"{len(dets)} msgs")
    check(("people + markers: YOLOX runs on the same frames (no person in a noise image)" if PEOPLE else
           "markers-only mode: person detections still published (empty), so the UI keeps its frames"),
          len(people) >= 10 and all(not d.detections for d in people), f"{len(people)} msgs")
    dets.clear(); tracks.clear()
    run(scene, 2.0)
    ids = sorted({k.marker_id for d in dets for k in d.markers})
    check("person_detector decodes both markers from the camera image (rgb8, upside-down mount)",
          ids == [7, 9] and dets[-1].dictionary == "DICT_4X4_50", str(ids))
    last = tracks[-1] if tracks else None
    trk = {k.marker_id: k for k in last.tracks} if last else {}
    check("marker_tracker tracks marker 7 only (9 is not in marker_ids)", sorted(trk) == [7], str(sorted(trk)))
    k7 = trk.get(7)
    if k7 is not None:
        r_true, b_true = math.hypot(X, Y), math.degrees(math.atan2(Y, X))
        check("marker 7 confirmed, range within 3 % and bearing within 1 deg of the truth",
              k7.confirmed and abs(k7.range_m - r_true) / r_true < 0.03 and abs(k7.bearing_deg - b_true) < 1.0,
              f"{k7.range_m:.2f} m / {k7.bearing_deg:+.1f} deg vs {r_true:.2f} m / {b_true:+.1f} deg")
        check("outline is in native image pixels, id = 1000000 + 7, rotation passed to the page",
              k7.id == 1000007 and last.image_rotation_deg == 180
              and all(0 <= v <= W for v in k7.corners), str(list(np.round(k7.corners, 0))))
    else:
        results.extend([False, False])
    check("nothing is a target before the operator selects", not targets, f"{len(targets)} targets")
    for _ in range(3):
        p_sel.publish(Int32(data=1000007))
    run(scene, 1.5)
    check("selected -> /follow/target at the marker (base_footprint)",
          len(targets) >= 8 and all(math.hypot(x - X, y - Y) < 0.12 for x, y in targets),
          f"{len(targets)} targets, last {targets[-1] if targets else None}")
    targets.clear()
    run(empty, 1.5)
    late = len(targets)
    check("marker gone: targets stop within 0.2 s (nothing is extrapolated)", late <= 2, f"{late} after it vanished")
    check("... state SEARCHING", bool(tracks) and tracks[-1].target_state == "SEARCHING",
          tracks[-1].target_state if tracks else "no tracks")
    targets.clear()
    run(scene, 1.0)
    check("marker back: target again, same id",
          len(targets) >= 5 and bool(tracks) and tracks[-1].target_state == "TRACKING",
          f"{len(targets)} targets, {tracks[-1].target_state if tracks else 'no tracks'}")
    both = [v for v in arrive.values() if "ui" in v and "people" in v]
    check("the operator's JPEG is published for every processed frame, same stamp as its detections",
          len(both) >= 0.9 * len([v for v in arrive.values() if "people" in v]) and len(both) >= 30,
          f"{len(both)} frames with both")
    if PEOPLE:
        first = sum(v["ui"] <= v["people"] for v in both)
        check("VIDEO FIRST: with YOLOX running, the JPEG goes out BEFORE the person detections",
              first >= 0.9 * len(both), f"{first}/{len(both)} frames")
    check("both nodes still running", all(p.poll() is None for p in procs))
finally:
    for p in procs:
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
    time.sleep(1.5)
    for name in ("person_detector", "marker_tracker"):   # incl. a clean Ctrl-C shutdown
        log = open(f"/tmp/marker_pipe_{name}.log").read()
        check(f"no errors in the {name} log", "rror" not in log and "Traceback" not in log, log[-200:])
    for p in procs:
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGKILL)
    n.destroy_node()
    rclpy.try_shutdown()
print(f"\n{sum(results)}/{len(results)} passed", flush=True)
os._exit(0 if results and all(results) else 1)
