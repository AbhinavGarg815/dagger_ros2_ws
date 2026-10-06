#!/usr/bin/env python3
"""Offline test of waypoint_queue (Foxglove clicks -> NavigateThroughPoses). NO ROVER.

    python3 tools/test_waypoint_queue.py

Runs in ROS_DOMAIN_ID 97 with a stub /navigate_through_poses action server that records the goal.
Clicks in odom are queued and drawn on /waypoints/path, a click in base_footprint or (mid-route)
in map is refused, go sends all waypoints in order with stamp 0 and the odom tree, clear empties
the queue, go with nothing queued sends nothing.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "97"
import atexit, signal, subprocess, sys, time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
from std_msgs.msg import Empty

SRC = "export ROS_DOMAIN_ID=97 && source /opt/ros/humble/setup.bash && source /home/jetson/ros2_ws/install/setup.bash"
proc = subprocess.Popen(["bash", "-lc", f"{SRC} && exec ros2 run dagger_bringup waypoint_queue"],
                        stdout=open("/tmp/waypoint_queue_test.log", "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)


@atexit.register
def stop():
    for sig in (signal.SIGINT, signal.SIGKILL):
        try: os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError: pass
        time.sleep(1.0)


rclpy.init(); n = Node("waypoint_queue_test")
goals, paths = [], []
def execute(gh):
    goals.append(gh.request); gh.succeed(); return NavigateThroughPoses.Result()
ActionServer(n, NavigateThroughPoses, "/navigate_through_poses", execute)
n.create_subscription(Path, "/waypoints/path", lambda m: paths.append(m),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
click = n.create_publisher(PoseStamped, "/waypoint_ui", 10)
go = n.create_publisher(Empty, "/waypoints/go", 10)
clear = n.create_publisher(Empty, "/waypoints/clear", 10)
checks = []
def check(name, ok, detail=""):
    checks.append(bool(ok)); print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
def spin(sec):
    end = time.time() + sec
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.02)
def send_click(x, y, frame="odom"):
    p = PoseStamped(); p.header.frame_id = frame; p.header.stamp.sec = 1234
    p.pose.position.x, p.pose.position.y = float(x), float(y); p.pose.orientation.w = 1.0
    click.publish(p); spin(0.4)

end = time.time() + 15
while time.time() < end and n.count_subscribers("/waypoint_ui") == 0: spin(0.2)
spin(0.5)
for xy in ((1, 0), (2, 1), (3, 3)): send_click(*xy)
check("3 clicks in odom -> /waypoints/path shows 3 waypoints", paths and len(paths[-1].poses) == 3 and paths[-1].header.frame_id == "odom",
      f"{len(paths[-1].poses) if paths else 0} poses")
send_click(9, 9, "base_footprint"); send_click(8, 8, "map")
check("a click in base_footprint and a map click mid-route are refused", len(paths[-1].poses) == 3)
go.publish(Empty()); spin(2.0)
g = goals[-1] if goals else None
check("go -> one NavigateThroughPoses goal with the 3 waypoints in order",
      g is not None and [(p.pose.position.x, p.pose.position.y) for p in g.poses] == [(1, 0), (2, 1), (3, 3)])
check("... frame odom, stamp 0 (latest transform), the odom tree requested",
      g is not None and all(p.header.frame_id == "odom" and p.header.stamp.sec == 0 for p in g.poses)
      and g.behavior_tree.endswith("dagger_navigate_through_poses_odom.xml"), g.behavior_tree if g else "")
clear.publish(Empty()); spin(0.5)
check("clear -> /waypoints/path empty", len(paths[-1].poses) == 0)
n_before = len(goals); go.publish(Empty()); spin(1.0)
check("go with nothing queued sends nothing", len(goals) == n_before)
send_click(5, 5, "map"); go.publish(Empty()); spin(2.0)
check("after clear a map route works and uses the default tree", len(goals) == n_before + 1 and goals[-1].poses[0].header.frame_id == "map"
      and goals[-1].behavior_tree == "")
print(f"\n{sum(checks)}/{len(checks)} passed")
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if all(checks) else 1)
