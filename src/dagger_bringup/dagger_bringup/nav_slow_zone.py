#!/usr/bin/env python3
"""Slow zone for the Nav2 command path that leaves in-place turns alone (Q70).

    /cmd_vel_nav_raw (controller_server, behavior_server) -> [this] -> /cmd_vel_nav_slow
        -> collision_monitor (stop zone + footprint approach) -> /cmd_vel_nav

Humble's collision monitor scales the WHOLE twist inside a slowdown polygon, so anything
0.85-2.6 m ahead cut in-place turns to x0.4: 0.45 -> 0.18 rad/s, and the controller's
0.12 rad/s start-of-turn to 0.05 -- below the tracks' turn deadband. The turn never
started, the measured rate stayed 0, the command never grew, and the progress checker
aborted the goal (live 2026-10-03, bag detour_fox1_132302).

Here the same zone, ratio and trigger as the old PolygonSlow apply while the rover moves
forward (whole twist scaled, so an arc keeps its curvature), but a pure in-place turn
(|linear| below turn_speed_limit) passes untouched: the monitor's stop zone and footprint
approach still guard turns. A stale /scan_safety counts as occupied.
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class NavSlowZone(Node):

    def __init__(self):
        super().__init__("nav_slow_zone")
        self.declare_parameter("cmd_in", "/cmd_vel_nav_raw")
        self.declare_parameter("cmd_out", "/cmd_vel_nav_slow")
        self.declare_parameter("scan_topic", "/scan_safety")
        self.declare_parameter("zone", [0.85, 2.60, 0.85])   # x_min, x_max, half width (m)
        self.declare_parameter("ratio", 0.4)
        self.declare_parameter("max_points", 3)       # slowed with MORE rays than this inside
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("turn_speed_limit", 0.02)     # m/s: below this it is a turn
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.x0, self.x1, self.hw = (float(v) for v in g("zone"))
        self.ratio = float(g("ratio"))
        self.max_points = int(g("max_points"))
        self.timeout = float(g("scan_timeout"))
        self.turn_v = float(g("turn_speed_limit"))
        self.pts = 0
        self.scan_time = 0.0
        self.state = None
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Twist, g("cmd_out"), 10)
        self.create_subscription(LaserScan, g("scan_topic"), self.on_scan, be)
        self.create_subscription(Twist, g("cmd_in"), self.on_cmd, 10)
        self.get_logger().info(
            f"nav_slow_zone: {g('cmd_in')} -> {g('cmd_out')}, zone x {self.x0}-{self.x1} m "
            f"+-{self.hw} m, x{self.ratio} while driving, in-place turns untouched")

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + np.arange(len(r), dtype=np.float32) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        x, y = r * np.cos(a), r * np.sin(a)
        self.pts = int((ok & (x >= self.x0) & (x <= self.x1) & (np.abs(y) <= self.hw)).sum())
        self.scan_time = time.monotonic()

    def on_cmd(self, msg):
        stale = time.monotonic() - self.scan_time > self.timeout
        occupied = stale or self.pts > self.max_points
        turning = abs(msg.linear.x) < self.turn_v and abs(msg.linear.y) < self.turn_v
        slow = occupied and not turning
        if slow:
            msg.linear.x *= self.ratio
            msg.linear.y *= self.ratio
            msg.angular.z *= self.ratio
        self.pub.publish(msg)
        state = ("x%g" % self.ratio) if slow else ("turn, untouched" if occupied else "clear")
        if state != self.state:
            self.state = state
            why = "scan stale" if stale else f"{self.pts} rays in the zone"
            self.get_logger().info(f"slow zone: {state} ({why})", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = NavSlowZone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
