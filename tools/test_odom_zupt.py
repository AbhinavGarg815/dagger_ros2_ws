#!/usr/bin/env python3
"""Replay a recorded stop through the REAL fcu_odom_relay, with and without the
neutral-PWM zero-velocity update (P10). NO ROVER: ROS_DOMAIN_ID 42.

    python3 tools/test_odom_zupt.py [~/dagger_bags/pf_stopdist_cruise04]

Measures how far the relay's published velocity integrates to over the 10 s
after the tracks return to neutral -- i.e. the false slide the costmap sees.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "42"
import signal, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCOut

BAG = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/dagger_bags/pf_stopdist_cruise04")
SRC = ("export ROS_DOMAIN_ID=42 && source /opt/ros/humble/setup.bash && "
       "source /home/jetson/ros2_ws/install/setup.bash")


def load():
    """velocity_body -> the relay's Odometry input, plus rc/out, at recorded times."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import TwistStamped
    r = SequentialReader()
    r.open(StorageOptions(uri=BAG, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    ev = []
    while r.has_next():
        t, d, ts = r.read_next()
        if t == "/mavros/local_position/velocity_body":
            ev.append((ts / 1e9, "v", deserialize_message(d, TwistStamped).twist))
        elif t == "/mavros/rc/out":
            ev.append((ts / 1e9, "rc", deserialize_message(d, RCOut)))
    ev.sort(key=lambda e: e[0])
    return ev


EV = None


def run(zupt):
    relay = subprocess.Popen(["bash", "-lc", f"{SRC} && exec ros2 run dagger_bringup fcu_odom_relay "
        f"--ros-args -p publish_tf:=false -p zero_velocity_deadband:=0.02 -p pivot_yaw_rate_gate:=0.25 "
        f"-p zero_lateral_velocity:=true -p zupt_neutral_pwm:={zupt} "
        f"-r /mavros/local_position/odom:=/test/odom_in -r /mavros/rc/out:=/test/rc_out "
        f"-r /odometry/filtered:=/test/odom_out"],
        stdout=open(f"/tmp/zupt_relay_{zupt}.log", "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    rclpy.init(); n = Node("zupt_test")
    rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
    be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
    p_od = n.create_publisher(Odometry, "/test/odom_in", be)
    p_rc = n.create_publisher(RCOut, "/test/rc_out", be)
    out = []
    n.create_subscription(Odometry, "/test/odom_out",
                          lambda m: out.append((time.time(), m.twist.twist.linear.x)), rel)
    time.sleep(4)
    t0_bag, t0 = EV[0][0], time.time()
    rc_wall = []
    for tb, kind, m in EV:
        while time.time() - t0 < tb - t0_bag:
            rclpy.spin_once(n, timeout_sec=0.002)
        if kind == "v":
            o = Odometry(); o.twist.twist = m; o.pose.pose.orientation.w = 1.0
            p_od.publish(o)
        else:
            p_rc.publish(m); rc_wall.append((time.time(), m.channels[0], m.channels[2]))
    end = time.time() + 1
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
    try: os.killpg(os.getpgid(relay.pid), signal.SIGINT)
    except ProcessLookupError: pass
    time.sleep(1)
    n.destroy_node(); rclpy.shutdown()
    # the stop: first powered -> neutral transition that then stays neutral for 10 s
    pw = [(t, abs(l - 1500) > 25 or abs(r - 1500) > 25) for t, l, r in rc_wall]
    t_stop = None
    for (ta, a), (tb_, b) in zip(pw, pw[1:]):
        if a and not b and all(not x for t, x in pw if tb_ <= t <= tb_ + 10):
            t_stop = tb_; break
    if t_stop is None or not out:
        return None
    def travel(a_, b_):
        seg = [(t, v) for t, v in out if t_stop + a_ <= t <= t_stop + b_]
        return sum(abs(p[1]) * (q[0] - p[0]) for p, q in zip(seg, seg[1:]))
    # 0-1.0 s (the hold) contains the REAL coasting (N8: at rest 0.70 s after throttle cut) and must
    # survive; 1.5-10 s the rover is stationary, so any travel there is false.
    return travel(0, 1.0), travel(1.5, 10)


EV = load()
print(f'replaying {len(EV)} messages from {os.path.basename(BAG)}')
base = run(0)
zupt = run(1500)
print(f"\nbag {os.path.basename(BAG)} -- 10 s after the tracks returned to neutral")
print("                 real coast (0-1.0 s)   FALSE slide at rest (1.5-10 s)")
for name, r in (("without ZUPT", base), ("with ZUPT   ", zupt)):
    if r is None: print(f"  {name}: no data"); continue
    print(f"  {name}      {r[0]:.3f} m               {r[1]:.3f} m")
ok1 = bool(base and zupt and base[1] > 0.1 and zupt[1] < 0.01)
ok2 = bool(base and zupt and abs(base[0] - zupt[0]) < 0.02)
print(f"\n  [{'PASS' if ok1 else 'FAIL'}] ZUPT removes the false slide once the rover is at rest")
print(f"  [{'PASS' if ok2 else 'FAIL'}] ZUPT leaves genuine coasting untouched")
ok = ok1 and ok2
sys.exit(0 if ok else 1)
