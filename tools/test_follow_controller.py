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
from sensor_msgs.msg import LaserScan
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
sel = n.create_publisher(Int32, "/follow/select", 10)
scan_pub = n.create_publisher(LaserScan, "/scan_safety", be)
obstacle = [False]           # a post 1.8 m ahead in the slow zone?


def scan():
    s = LaserScan(); s.header.frame_id = "base_footprint"
    s.header.stamp = n.get_clock().now().to_msg()
    s.angle_min, s.angle_increment, s.range_min, s.range_max = -math.pi, 0.0087, 1.0, 20.0
    nb = int(2 * math.pi / 0.0087)
    s.ranges = [1.8 if obstacle[0] and abs(-math.pi + i * 0.0087) < 0.08 else float("inf")
                for i in range(nb)]
    scan_pub.publish(s)


n.create_timer(0.1, scan)
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
obstacle[0] = True
spin(0.3)                    # let a scan with the obstacle arrive (scans are 10 Hz)
r_slowzone = phase("5 m ahead, obstacle IN the slow zone", 3.0, 5.0)
r_slowturn = phase("  ...target at 60 deg (turn in place)", 3.0, *pol(4.0, 60))
obstacle[0] = False
r_clear = phase("zone clear again", 3.0, 5.0)
r_id1 = phase("following track 7", 2.0, 4.5, 0.0, track=7)
r_id2 = phase("track id changes to 8", 2.0, 4.5, 0.0, track=8)
r_id2b = phase("  ...still track 8", 2.0, 4.5, 0.0, track=8)
enable()
phase("1.3 m at +45 deg (settle)", 0.3, *pol(1.3, 45))
r_hug = phase("  ...1.3 m at +45 deg (inside min_turn_range)", 2.7, *pol(1.3, 45))
r_hugb = phase("1.7 m at +45 deg (inside the 0.2 m release band)", 1.5, *pol(1.7, 45))
r_hug2 = phase("2.5 m at +45 deg (outside it)", 3.0, *pol(2.5, 45))
phase("back to 1.3 m at +60 deg (settle)", 0.3, *pol(1.3, 60))
r_hug3 = phase("  ...1.3 m at +60 deg", 1.2, *pol(1.3, 60))
r_hugl = phase("lost there: search window", 2.0)


def select(i):
    for _ in range(3):
        sel.publish(Int32(data=i)); spin(0.03)


# the operator's selection ends the session at once (live aruco2: CLEAR -> 29 s of search turns)
spin(7.0)                                        # let the search above expire -> disabled
enable()
r_cf = phase("following track 7 at 4 m, 60 deg", 1.5, *pol(4.0, 60), track=7)
select(-1)
phase("CLEAR while following (settle)", 0.3, *pol(4.0, 60), track=7)
r_clr = phase("  ...target still published", 1.7, *pol(4.0, 60), track=7)
enable()
phase("following track 7 again", 1.5, *pol(4.0, 60), track=7)
r_cs = phase("target lost: searching", 1.5)
select(-1)
phase("CLEAR during the search (settle)", 0.3)
r_clrs = phase("  ...search window still open", 2.0)
enable()
phase("following track 7 again", 1.5, *pol(4.0, 60), track=7)
select(9)
phase("operator clicks #9, no target from it yet (settle)", 0.3, *pol(4.0, 60), track=7)
r_oth = phase("  ...old target still published", 1.7, *pol(4.0, 60), track=7)
enable()
phase("following track 7 again", 1.5, *pol(4.0, 60), track=7)
select(7)
phase("operator clicks #7 again, the one followed (settle)", 0.3, *pol(4.0, 60), track=7)
r_same = phase("  ...keeps following", 1.5, *pol(4.0, 60), track=7)

def quiet(r):
    """No motion commanded: silent, or zeros only."""
    return r is None or (abs(r[0]) < 1e-6 and abs(r[1]) < 1e-6 and abs(r[2]) < 1e-6 and abs(r[3]) < 1e-6)


checks = [
    ("silent while disabled", r_dis is None),
    ("drives forward when too far", r_fwd and r_fwd[1] > 0.05),
    ("respects max_linear 0.6 (the default)", r_fwd and r_fwd[1] <= 0.6 + 1e-6),
    ("settles to zero in the deadband (after ramping down)", r_dead and r_dead[0] < 1e-6),
    ("never reverses when too close", r_close and r_close[0] >= 0.0),
    ("turns IN PLACE for a target at 60 deg (v = 0)", r_side and r_side[1] < 1e-6 and r_side[3] > 0.3),
    ("search is rotation only", r_srch and abs(r_srch[0]) < 1e-6 and abs(r_srch[1]) < 1e-6
     and r_srch[3] > 0.2),
    ("stops commanding once the search expires", r_after is None or r_after[1] < 1e-6),
    ("does not resume on its own", r_noauto is None),
    ("resumes after explicit re-enable", r_reen and r_reen[1] > 0.05),
    ("no yaw chatter when bearing dithers across 5 deg", toggles <= 2),
    ("slow zone occupied: forward speed <= 0.4 x", r_slowzone and r_slowzone[1] <= 0.4 * 0.6 + 1e-6
     and r_slowzone[1] > 0.05),
    ("slow zone occupied: turning in place NOT slowed", r_slowturn and r_slowturn[3] > 0.3),
    ("zone clear: full speed again", r_clear and r_clear[1] > 0.4),
    ("DISABLES when the track id changes (different person)",
     r_id2b is None or (r_id2b[1] < 1e-6 and abs(r_id2b[3]) < 1e-6)),
    ("1.3 m away at 45 deg (inside the corner sweep + margin): no turn, no drive",
     r_hug and r_hug[1] < 1e-6 and abs(r_hug[2]) < 1e-6 and abs(r_hug[3]) < 1e-6),
    ("... 1.7 m (on the 1.6 m line + < 0.2 m): still holding, no toggling",
     r_hugb and r_hugb[1] < 1e-6 and abs(r_hugb[2]) < 1e-6 and abs(r_hugb[3]) < 1e-6),
    ("2.5 m away at 45 deg: turns in place again", r_hug2 and r_hug2[1] < 1e-6 and r_hug2[3] > 0.3),
    ("... back to 1.3 m: stops turning", r_hug3 and abs(r_hug3[2]) < 1e-6 and abs(r_hug3[3]) < 1e-6),
    ("lost 1.3 m away: the search does NOT turn",
     r_hugl is None or (abs(r_hugl[2]) < 1e-6 and abs(r_hugl[3]) < 1e-6)),
    ("(following at 60 deg: turning, as a reference)", r_cf and r_cf[3] > 0.3),
    ("CLEAR while following: stops at once, although the target is still there",
     quiet(r_clr)),
    ("(target lost: search turns, as a reference)", r_cs and max(abs(r_cs[2]), abs(r_cs[3])) > 0.2),
    ("CLEAR during the search: the turning stops (live aruco2: it went on for 29 s)", quiet(r_clrs)),
    ("click on another target while following: stops before that target even appears", quiet(r_oth)),
    ("click on the target already followed: keeps following", r_same and r_same[3] > 0.3),
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
