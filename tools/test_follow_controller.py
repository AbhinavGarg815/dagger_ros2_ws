#!/usr/bin/env python3
"""Offline safety test for follow_controller. NO ROVER, NO FCU.

    python3 tools/test_follow_controller.py

Runs in ROS_DOMAIN_ID 42 (set before rclpy is imported, inherited by the node it
starts), so it cannot reach a live stack: a second follow_controller on the live
domain WILL drive an armed rover. Feeds synthetic targets, enable and track ids,
and checks what the controller commands.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "42"
import math, signal, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Int32

SRC = ("export ROS_DOMAIN_ID=42 && source /opt/ros/humble/setup.bash && "
       "source /home/jetson/ros2_ws/install/setup.bash")
ctl = subprocess.Popen(["bash", "-lc", f"{SRC} && exec ros2 run dagger_bringup follow_controller"],
                       stdout=open("/tmp/follow_ctrl_test.log", "w"), stderr=subprocess.STDOUT,
                       preexec_fn=os.setsid)
rclpy.init(); n = Node("ctrl_test")
be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
tgt = n.create_publisher(PoseStamped, "/follow/target", be)
en = n.create_publisher(Bool, "/follow/enable", 10)
tid = n.create_publisher(Int32, "/follow/track_id", 10)
cmds = []
n.create_subscription(Twist, "/cmd_vel_follow_raw", lambda m: cmds.append(m), 10)


def spin(d):
    t = time.time() + d
    while time.time() < t:
        rclpy.spin_once(n, timeout_sec=0.005)


def phase(name, dur, x=None, y=0.0, track=None):
    cmds.clear()
    t = time.time() + dur
    while time.time() < t:
        if x is not None:
            if track is not None:
                tid.publish(Int32(data=track))
            p = PoseStamped(); p.header.frame_id = "base_footprint"
            p.header.stamp = n.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            tgt.publish(p)
        spin(0.05)
    if not cmds:
        print(f"  {name:44s} -> no messages"); return None
    vs = [c.linear.x for c in cmds]; ws = [c.angular.z for c in cmds]
    print(f"  {name:44s} -> {len(cmds):3d} msgs v[{min(vs):+.3f},{max(vs):+.3f}] "
          f"w[{min(ws):+.3f},{max(ws):+.3f}]")
    return min(vs), max(vs), min(ws), max(ws)


def enable():
    for _ in range(5):
        en.publish(Bool(data=True)); spin(0.05)
    spin(0.3)


def pol(r, deg):
    return r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))


time.sleep(7)
r_dis = phase("disabled, target 5 m ahead", 2.0, 5.0)
enable()
r_fwd = phase("following, 5 m ahead", 3.0, 5.0)
r_dead = phase("3.0 m ahead (deadband)", 3.0, 3.0)
r_close = phase("1.5 m ahead (too close)", 3.0, 1.5)
r_side = phase("4 m at +60 deg (beside the rover)", 3.0, *pol(4.0, 60))
r_loss = phase("lost off-axis: search window", 0.9); r_srch = phase("  ...search", 1.8)
r_after = phase("  ...after search", 7.0)
r_noauto = phase("target back, NOT re-enabled", 2.0, 4.5)
enable()
r_reen = phase("after explicit re-enable", 3.0, 4.5)
# bearing dithering across the 5 deg threshold: 4.5 / 5.5 / 4.5 / ...
cmds.clear()
for k in range(60):
    p = PoseStamped(); p.header.frame_id = "base_footprint"
    p.header.stamp = n.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = pol(4.5, 5.5 if k % 2 else 4.5)
    tgt.publish(p); spin(0.05)
ws = [abs(c.angular.z) > 1e-3 for c in cmds]
toggles = sum(1 for a, b in zip(ws, ws[1:]) if a != b)
print(f"  {'bearing dithering 4.5/5.5 deg':44s} -> yaw on/off toggles: {toggles}")
r_id1 = phase("following track 7", 2.0, 4.5, 0.0, track=7)
r_id2 = phase("track id changes to 8", 2.0, 4.5, 0.0, track=8)
r_id2b = phase("  ...still track 8", 2.0, 4.5, 0.0, track=8)

checks = [
    ("silent while disabled", r_dis is None),
    ("drives forward when too far", r_fwd and r_fwd[1] > 0.05),
    ("respects max_linear 0.4", r_fwd and r_fwd[1] <= 0.4 + 1e-6),
    ("settles to zero in the deadband (after ramping down)", r_dead and r_dead[0] < 1e-6),
    ("never reverses when too close", r_close and r_close[0] >= 0.0),
    ("turns IN PLACE for a target at 60 deg (v = 0)", r_side and r_side[1] < 1e-6 and r_side[3] > 0.3),
    ("search is rotation only", r_srch and abs(r_srch[0]) < 1e-6 and abs(r_srch[1]) < 1e-6
     and r_srch[3] > 0.2),
    ("stops commanding once the search expires", r_after is None or r_after[1] < 1e-6),
    ("does not resume on its own", r_noauto is None),
    ("resumes after explicit re-enable", r_reen and r_reen[1] > 0.05),
    ("no yaw chatter when bearing dithers across 5 deg", toggles <= 2),
    ("DISABLES when the track id changes (different person)",
     r_id2b is None or (r_id2b[1] < 1e-6 and abs(r_id2b[3]) < 1e-6)),
]
print()
for name, ok in checks:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
print(f"\n{sum(bool(ok) for _, ok in checks)}/{len(checks)} passed")
try: os.killpg(os.getpgid(ctl.pid), signal.SIGINT)
except ProcessLookupError: pass
time.sleep(2)
try: os.killpg(os.getpgid(ctl.pid), signal.SIGKILL)
except ProcessLookupError: pass
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if all(ok for _, ok in checks) else 1)
