#!/usr/bin/env python3
"""Offline test of extnav_relay (LiDAR odometry -> ArduPilot ExternalNav, Q80) and of the MAVROS
vision plugins it feeds. NO ROVER, NO FCU.

    python3 tools/test_extnav_relay.py

Runs in ROS_DOMAIN_ID 96. 1) extnav_relay: an Odometry in, the pose passed through unchanged
(same stamp), the velocity rotated from base_footprint into the odometry frame. 2) mavros_node
with T1's plugin list (incl. vision_pose, vision_speed) and no FCU: both plugins load and
subscribe to the topics the relay publishes.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "96"
import atexit, math, signal, subprocess, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

SRC = "export ROS_DOMAIN_ID=96 && source /opt/ros/humble/setup.bash && source /home/jetson/ros2_ws/install/setup.bash"
PLUGINS = ('["sys_status", "sys_time", "command", "param", "rc_io", "imu", "local_position", "global_position", '
           '"home_position", "gps_status", "setpoint_velocity", "setpoint_raw", "waypoint", "vision_pose", "vision_speed"]')
procs = [subprocess.Popen(["bash", "-lc", f"{SRC} && exec ros2 run dagger_bringup extnav_relay"],
                          stdout=open("/tmp/extnav_relay_test.log", "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid),
         subprocess.Popen(["bash", "-lc", f"{SRC} && exec ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14996@ "
                           f"-p 'plugin_denylist:=[\"*\"]' -p 'plugin_allowlist:={PLUGINS}'"],
                          stdout=open("/tmp/extnav_mavros_test.log", "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)]


@atexit.register
def stop():
    for sig in (signal.SIGINT, signal.SIGKILL):
        for p in procs:
            try: os.killpg(os.getpgid(p.pid), sig)
            except ProcessLookupError: pass
        time.sleep(1.0)


rclpy.init(); n = Node("extnav_relay_test")
got = {"pose": [], "speed": []}
n.create_subscription(PoseStamped, "/mavros/vision_pose/pose", lambda m: got["pose"].append(m), 10)
n.create_subscription(TwistStamped, "/mavros/vision_speed/speed_twist", lambda m: got["speed"].append(m), 10)
pub = n.create_publisher(Odometry, "/lidar_odom/odometry", 10)
checks = []
def check(name, ok, detail=""):
    checks.append(bool(ok)); print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def send(x, y, yaw, vx, vy, sec):
    o = Odometry(); o.header.frame_id = "lidar_odom"; o.child_frame_id = "base_footprint"
    o.header.stamp.sec = sec
    o.pose.pose.position.x, o.pose.pose.position.y = x, y
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    o.twist.twist.linear.x, o.twist.twist.linear.y = vx, vy
    got["pose"].clear(); got["speed"].clear(); pub.publish(o)
    end = time.time() + 2.0
    while time.time() < end and not (got["pose"] and got["speed"]):
        rclpy.spin_once(n, timeout_sec=0.02)
    return (got["pose"][-1] if got["pose"] else None), (got["speed"][-1] if got["speed"] else None)


end = time.time() + 15
while time.time() < end and n.count_subscribers("/lidar_odom/odometry") == 0:
    rclpy.spin_once(n, timeout_sec=0.1)
print("1. extnav_relay")
p, s = send(1.0, 2.0, math.pi / 2, 0.5, 0.0, 100)
check("pose passed through (position, yaw, stamp)", p and abs(p.pose.position.x - 1) < 1e-9 and abs(p.pose.position.y - 2) < 1e-9
      and abs(2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w) - math.pi / 2) < 1e-9 and p.header.stamp.sec == 100)
check("heading +90 deg, 0.5 m/s forward -> velocity (0, +0.5) in the odometry frame",
      s and abs(s.twist.linear.x) < 1e-9 and abs(s.twist.linear.y - 0.5) < 1e-9 and s.header.stamp.sec == 100,
      f"({s.twist.linear.x:+.3f}, {s.twist.linear.y:+.3f})" if s else "none")
p, s = send(0.0, 0.0, math.radians(30), 0.4, 0.1, 101)
ex, ey = 0.4 * math.cos(math.radians(30)) - 0.1 * math.sin(math.radians(30)), 0.4 * math.sin(math.radians(30)) + 0.1 * math.cos(math.radians(30))
check("heading 30 deg, body (0.4, 0.1) -> rotated into the odometry frame",
      s and abs(s.twist.linear.x - ex) < 1e-9 and abs(s.twist.linear.y - ey) < 1e-9, f"({s.twist.linear.x:.3f}, {s.twist.linear.y:.3f})" if s else "none")

print("2. mavros (no FCU) with T1's plugin list")
end = time.time() + 20
while time.time() < end and (n.count_subscribers("/mavros/vision_pose/pose") == 0 or n.count_subscribers("/mavros/vision_speed/speed_twist") == 0):
    rclpy.spin_once(n, timeout_sec=0.1)
log = open("/tmp/extnav_mavros_test.log").read()
check("vision_pose and vision_speed plugins initialised", "vision_pose" in log and "vision_speed" in log and "initialized" in log)
check("mavros subscribes /mavros/vision_pose/pose and /mavros/vision_speed/speed_twist",
      n.count_subscribers("/mavros/vision_pose/pose") >= 1 and n.count_subscribers("/mavros/vision_speed/speed_twist") >= 1,
      f"{n.count_subscribers('/mavros/vision_pose/pose')} / {n.count_subscribers('/mavros/vision_speed/speed_twist')} subscribers")
print(f"\n{sum(checks)}/{len(checks)} passed")
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if all(checks) else 1)
