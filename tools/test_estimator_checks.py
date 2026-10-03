#!/usr/bin/env python3
"""Offline test of the FCU-estimator guards added after Q67. NO ROVER.

    python3 tools/test_estimator_checks.py

Runs in ROS_DOMAIN_ID 79 with a fake FCU (mavros topics) replaying 2026-10-03:
GNSS heading valid in 8 % of messages and EK3 reporting 250 m/s while parked.
  1. tools/preflight.py --no-config --no-nav2 must FAIL gps-yaw and fcu-estimator,
     and must not fail them with a healthy fake FCU;
  2. fcu_odom_relay (max_plausible_speed 2.0, ZUPT on) must pass 0.4 m/s, drop 250 m/s
     while the tracks drive, and publish its own zero while they sit at neutral.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "79"
import atexit, math, signal, subprocess, sys, threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import GPSRAW, RCOut, State
from nav_msgs.msg import Odometry

WS = "/home/jetson/ros2_ws"
SRC = f"export ROS_DOMAIN_ID=79 && source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash"
rclpy.init(); n = Node("fake_fcu")
pub_gps = n.create_publisher(GPSRAW, "/mavros/gpsstatus/gps1/raw", qos_profile_sensor_data)
pub_odom = n.create_publisher(Odometry, "/mavros/local_position/odom", qos_profile_sensor_data)
pub_rc = n.create_publisher(RCOut, "/mavros/rc/out", qos_profile_sensor_data)
pub_state = n.create_publisher(State, "/mavros/state", qos_profile_sensor_data)
fake = {"yaw_every": 12, "speed": 250.0, "neutral": True, "k": 0, "x": 0.0}


def tick():                                    # 20 Hz fake FCU
    f = fake; f["k"] += 1
    o = Odometry(); o.header.stamp = n.get_clock().now().to_msg(); o.header.frame_id = "map"
    f["x"] += f["speed"] * 0.05
    o.pose.pose.position.x = f["x"]; o.pose.pose.orientation.w = 1.0
    o.twist.twist.linear.x = f["speed"]
    pub_odom.publish(o)
    rc = RCOut(); rc.channels = [1500, 1500, 1500, 1500] if f["neutral"] else [1700, 1500, 1700, 1500]
    pub_rc.publish(rc)
    if f["k"] % 4 == 0:                        # 5 Hz GNSS
        g = GPSRAW(); g.fix_type = 3; g.satellites_visible = 26; g.eph = 65
        g.yaw = 5100 if f["k"] % (4 * f["yaw_every"]) == 0 else 65535
        pub_gps.publish(g)
    if f["k"] % 20 == 0:
        s = State(); s.connected = True; s.mode = "HOLD"; pub_state.publish(s)


n.create_timer(0.05, tick)
stop = threading.Event()


def spin():
    while not stop.is_set():
        rclpy.spin_once(n, timeout_sec=0.02)


spinner = threading.Thread(target=spin, daemon=True); spinner.start()
checks = []


def check(name, ok, detail=""):
    checks.append(ok); print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def preflight():
    r = subprocess.run(["bash", "-lc", f"{SRC} && cd {WS} && python3 tools/preflight.py --no-config --no-nav2 -t 4"],
                       capture_output=True, text=True, timeout=60)
    last = [l for l in r.stdout.splitlines() if l.startswith("PREFLIGHT")]
    return r.stdout, (last[-1] if last else "")


print("1. preflight against a fake FCU")
fake.update(yaw_every=12, speed=250.0, neutral=True)
out, verdict = preflight()
check("diverged EK3 (heading 8 %, 250 m/s parked): FAIL gps-yaw", "gps-yaw" in verdict, verdict)
check("... and FAIL fcu-estimator", "fcu-estimator" in verdict)
fake.update(yaw_every=1, speed=0.01, neutral=True)
out, verdict = preflight()
check("healthy (heading 100 %, 0.01 m/s parked): no gps-yaw", "gps-yaw" not in verdict, verdict)
check("... and no fcu-estimator", "fcu-estimator" not in verdict)
fake.update(speed=250.0, neutral=False)
out, verdict = preflight()
check("tracks driving: the parked check is skipped, not failed", "skipped" in out and "fcu-estimator" not in verdict)

print("2. fcu_odom_relay plausibility gate")
relay = subprocess.Popen(["bash", "-lc", f"{SRC} && exec ros2 run dagger_bringup fcu_odom_relay --ros-args "
                          "-p output_topic:=/fcu/odom -p publish_tf:=false -p max_plausible_speed:=2.0 "
                          "-p zupt_neutral_pwm:=1500 -p zero_lateral_velocity:=true"],
                         stdout=open("/tmp/estimator_checks_relay.log", "w"), stderr=subprocess.STDOUT,
                         preexec_fn=os.setsid)
atexit.register(lambda: os.killpg(os.getpgid(relay.pid), signal.SIGKILL))
got = []
n.create_subscription(Odometry, "/fcu/odom", lambda m: got.append(m.twist.twist.linear.x),
                      QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))


def phase(speed, neutral, secs=2.5):
    fake.update(speed=speed, neutral=neutral); time.sleep(0.5); got.clear(); time.sleep(secs)
    return list(got)


time.sleep(3.0)                                   # relay up
g = phase(0.4, False)
check("0.4 m/s while driving: passed through", len(g) > 20 and abs(sorted(g)[len(g) // 2] - 0.4) < 1e-6, f"{len(g)} msgs")
g = phase(250.0, False)
check("250 m/s while driving: dropped", len(g) == 0, f"{len(g)} msgs")
g = phase(250.0, True)
check("250 m/s parked (ZUPT): the relay's own zero is published", len(g) > 20 and max(abs(v) for v in g) == 0.0, f"{len(g)} msgs")
log = open("/tmp/estimator_checks_relay.log").read()
check("an ERROR names the implausible speed", "implausible" in log)

stop.set(); spinner.join()                       # stop spinning before shutting down
n.destroy_node(); rclpy.shutdown()
print(f"\n{sum(checks)}/{len(checks)} passed")
sys.exit(0 if all(checks) else 1)
