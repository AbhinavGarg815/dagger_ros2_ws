#!/usr/bin/env python3
"""Offline test of nav_slow_zone, the Nav2 slow zone that leaves in-place turns alone. NO ROVER.

    python3 tools/test_nav_slow_zone.py

Runs in ROS_DOMAIN_ID 82 (set before rclpy is imported). Feeds /scan_safety-style scans
and commands, checks what comes out: x0.4 on the whole twist while driving with more than
3 rays 0.85-2.6 m ahead (+-0.85 m) or a stale scan; in-place turns untouched -- the 0.12
rad/s start-of-turn that deadlocked live (Q70) must pass as 0.12, not 0.05.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "82"
import atexit, math, signal, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

SRC = ("export ROS_DOMAIN_ID=82 && source /opt/ros/humble/setup.bash && "
       "source /home/jetson/ros2_ws/install/setup.bash")
proc = subprocess.Popen(
    ["bash", "-lc", f"{SRC} && exec ros2 run dagger_bringup nav_slow_zone --ros-args "
     "-p cmd_in:=/t/cmd_in -p cmd_out:=/t/cmd_out -p scan_topic:=/t/scan"],
    stdout=open("/tmp/nav_slow_zone_test.log", "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)


@atexit.register
def stop_node():
    for sig in (signal.SIGINT, signal.SIGKILL):
        try: os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError: pass
        time.sleep(0.8)


rclpy.init(); n = Node("nav_slow_zone_test")
be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
scan_pub = n.create_publisher(LaserScan, "/t/scan", be)
cmd_pub = n.create_publisher(Twist, "/t/cmd_in", 10)
out = []
n.create_subscription(Twist, "/t/cmd_out", lambda m: out.append(m), 10)
INC, NB = 0.0087, 723


def scan(points):
    """points: list of (x, y) in base_footprint -> one ray each (nearest per bin)."""
    s = LaserScan(); s.header.frame_id = "base_footprint"
    s.angle_min, s.angle_max, s.angle_increment = -3.14159, 3.14159, INC
    s.range_min, s.range_max = 0.3, 20.0
    r = [math.inf] * NB
    for x, y in points:
        b = int((math.atan2(y, x) + 3.14159) / INC); r[b] = min(r[b], math.hypot(x, y))
    s.ranges = r
    return s


def wall(n_rays, x=1.5, y0=0.0):
    return [(x, y0 + (k - n_rays / 2) * x * INC * 1.01) for k in range(n_rays)]


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.01)


def through(points, vx, wz, scan_age=0.0):
    """Publish a scan (unless points is None), wait scan_age, send one command, return the output."""
    if points is not None:
        for _ in range(3):
            scan_pub.publish(scan(points)); spin(0.05)
    spin(scan_age)
    out.clear()
    c = Twist(); c.linear.x, c.angular.z = vx, wz
    cmd_pub.publish(c)
    end = time.time() + 2.0
    while time.time() < end and not out:
        rclpy.spin_once(n, timeout_sec=0.01)
    return (out[-1].linear.x, out[-1].angular.z) if out else None


checks = []
def check(name, got, want):
    ok = got is not None and abs(got[0] - want[0]) < 1e-6 and abs(got[1] - want[1]) < 1e-6
    checks.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}  (out {got}, expected {want})")


end = time.time() + 15                      # wait for the node
while time.time() < end and through([], 0.3, 0.0) is None:
    pass

check("clear, driving 0.4 / 0.1: untouched", through([], 0.4, 0.1), (0.4, 0.1))
check("10 rays 1.5 m ahead, driving 0.4 / 0.1: whole twist x0.4 (arc keeps its curvature)",
      through(wall(10), 0.4, 0.1), (0.4 * 0.4, 0.1 * 0.4))
check("10 rays 1.5 m ahead, turning in place 0.6 rad/s: untouched", through(wall(10), 0.0, 0.6), (0.0, 0.6))
check("10 rays 1.5 m ahead, start of a turn 0.12 rad/s (the live deadlock): untouched",
      through(wall(10), 0.0, -0.12), (0.0, -0.12))
check("exactly 3 rays in the zone: not slowed (needs MORE than 3)", through(wall(3), 0.4, 0.0), (0.4, 0.0))
check("4 rays in the zone: slowed", through(wall(4), 0.4, 0.0), (0.16, 0.0))
check("10 rays beside the zone (y 1.2 m): untouched", through(wall(10, 1.5, 1.2), 0.4, 0.0), (0.4, 0.0))
check("10 rays beyond the zone (x 3.0 m): untouched", through(wall(10, 3.0), 0.4, 0.0), (0.4, 0.0))
check("scan stale (0.7 s), driving: slowed", through(None, 0.4, 0.05, scan_age=0.7), (0.16, 0.02))
check("scan stale (0.7 s), turning in place: untouched", through(None, 0.0, 0.6, scan_age=0.0), (0.0, 0.6))
check("creeping 0.01 m/s with a turn counts as a turn: untouched", through(wall(10), 0.01, 0.5), (0.01, 0.5))

print(f"\n{sum(checks)}/{len(checks)} passed")
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if all(checks) else 1)
