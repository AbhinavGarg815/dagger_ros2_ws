#!/usr/bin/env python3
"""Keep pointcloud_to_laserscan alive: permanent /scan subscriber + wedge recovery.

Humble's pointcloud_to_laserscan subscribes to the cloud LAZILY (only while /scan
has subscribers). When the costmaps connect/disconnect during Nav2 startup the
count flaps 1 -> 0 -> 1 and the node can come back WEDGED: subscribed, but its tf
MessageFilter never releases a cloud -> /scan silent -> costmaps BLIND (seen twice
on 2026-09-14; an identical fresh instance published fine).

1. This node subscribes to /scan for its whole life, so the count never drops to
   0 once it is up (no flap).
2. If /scan stays silent for `silent_timeout` s while clouds keep arriving, it
   kills the p2l process; the launch respawns it (respawn=True) cleanly.
"""
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2


class ScanWatchdog(Node):

    def __init__(self):
        super().__init__("scan_watchdog")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cloud_topic", "/rslidar_points")
        self.declare_parameter("silent_timeout", 4.0)
        self.declare_parameter("process_pattern", "pointcloud_to_laserscan_node")
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.timeout = float(g("silent_timeout"))
        self.pattern = g("process_pattern")
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        now = time.monotonic()
        self.last_scan, self.last_cloud, self.last_kill = now, 0.0, 0.0
        self.create_subscription(LaserScan, g("scan_topic"), self.on_scan, be)
        # raw=True: only the arrival time matters, skip deserialising 0.8 MB clouds
        self.create_subscription(PointCloud2, g("cloud_topic"), self.on_cloud, be, raw=True)
        self.create_timer(1.0, self.check)
        self.get_logger().info(f"scan_watchdog: holding {g('scan_topic')}, timeout {self.timeout}s")

    def on_scan(self, _):
        self.last_scan = time.monotonic()

    def on_cloud(self, _):
        self.last_cloud = time.monotonic()

    def check(self):
        now = time.monotonic()
        cloud_live = now - self.last_cloud < 1.0
        silent = now - self.last_scan
        if cloud_live and silent > self.timeout and now - self.last_kill > 10.0:
            pids = subprocess.run(["pgrep", "-f", self.pattern], capture_output=True,
                                  text=True).stdout.split()
            own = str(os.getpid())
            pids = [p for p in pids if p != own]
            self.get_logger().error(
                f"/scan silent {silent:.1f}s while clouds arrive -> p2l WEDGED, restarting "
                f"(pids {pids}); costmaps are blind until it is back")
            for p in pids:
                try:
                    os.kill(int(p), signal.SIGTERM)
                except (ProcessLookupError, ValueError):
                    pass
            self.last_kill = now


def main(args=None):
    rclpy.init(args=args)
    node = ScanWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
