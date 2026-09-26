#!/usr/bin/env python3
"""Low obstacles only, for the costmap's low_obstacle_layer.

    /scan_low_all (non-ground 12-35 cm) + /scan_tall (non-ground 40-160 cm)
        -> /scan_low

WHY A SEPARATE LAYER NEEDS THIS. The 16-beam LiDAR, 1.12 m up, stops hitting a
low object once the rover is within ~3.3-4 m of it: the beams pass over. The
"nothing there" rays then CLEARED the object from the costmap -- measured live
on 2026-09-26, a box seen and marked lethal at 4.4 m was erased at 3.2 m, with
the rover still ~2.4 m short of it. The low_obstacle_layer only ray-clears
beyond 4 m, so a low object, once seen, is remembered while the rover closes.

What goes into that layer must therefore be things that are ONLY low: anything
with a tall return at the same bearing and range -- a person's legs under their
torso, the base of a wall -- belongs to the normal layer, which clears as
usual. Otherwise every passer-by would leave marks that persist near the rover.

Per bearing:
  low return, nothing tall there   -> the low range       (marked)
  low return with something tall   -> NaN                 (ignored: not low-only)
  no low return, something tall    -> NaN                 (cannot clear through it)
  neither                          -> inf                 (clears, beyond 4 m only)
Nav2 skips NaN ranges entirely and, with inf_is_valid, clears along inf rays.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class LowScanFilter(Node):

    def __init__(self):
        super().__init__("low_scan_filter")
        self.declare_parameter("low_topic", "/scan_low_all")
        self.declare_parameter("tall_topic", "/scan_tall")
        self.declare_parameter("out_topic", "/scan_low")
        # A tall return within this much of a low one, or nearer than it, means
        # the low return is part of something tall (legs under a torso).
        self.declare_parameter("tall_margin", 0.5)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.margin = float(g("tall_margin"))
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(LaserScan, g("out_topic"), be)
        self.low, self.tall = {}, {}
        self.create_subscription(LaserScan, g("low_topic"), lambda m: self.on(m, self.low), be)
        self.create_subscription(LaserScan, g("tall_topic"), lambda m: self.on(m, self.tall), be)
        self.n_pub = 0
        self.n_low = 0
        self.create_timer(30.0, self.report)
        self.get_logger().info(
            f"low_scan_filter: {g('low_topic')} minus anything tall in {g('tall_topic')} "
            f"-> {g('out_topic')}")

    @staticmethod
    def key(m):
        return (m.header.stamp.sec, m.header.stamp.nanosec)

    def on(self, msg, store):
        store[self.key(msg)] = msg
        # Both scans come from the same cloud, so they share its stamp exactly.
        k = self.key(msg)
        if k in self.low and k in self.tall:
            self.combine(self.low.pop(k), self.tall.pop(k))
        for d in (self.low, self.tall):         # drop anything never paired
            while len(d) > 10:
                d.pop(next(iter(d)))

    def combine(self, lo_msg, ta_msg):
        lo = np.asarray(lo_msg.ranges, dtype=np.float32)
        ta = np.asarray(ta_msg.ranges, dtype=np.float32)
        if lo.shape != ta.shape:
            self.get_logger().error("low/tall scans differ in size -- p2l configs must match",
                                    throttle_duration_sec=10.0)
            return
        lo_ok, ta_ok = np.isfinite(lo), np.isfinite(ta)
        out = np.full(lo.shape, np.inf, dtype=np.float32)
        tall_here = ta_ok & (~lo_ok | (ta < lo + self.margin))
        low_only = lo_ok & ~tall_here
        out[tall_here] = np.nan
        out[low_only] = lo[low_only]
        m = LaserScan()
        m.header = lo_msg.header
        m.angle_min, m.angle_max = lo_msg.angle_min, lo_msg.angle_max
        m.angle_increment, m.time_increment = lo_msg.angle_increment, lo_msg.time_increment
        m.scan_time = lo_msg.scan_time
        m.range_min, m.range_max = lo_msg.range_min, lo_msg.range_max
        m.ranges = out.tolist()
        self.pub.publish(m)
        self.n_pub += 1
        self.n_low += int(low_only.sum())

    def report(self):
        if self.n_pub:
            self.get_logger().info(f"{self.n_pub / 30.0:.1f} Hz, {self.n_low / max(1, self.n_pub):.1f} "
                                   f"low-only rays per scan")
        else:
            self.get_logger().warn("no paired low/tall scans in 30 s")
        self.n_pub = self.n_low = 0


def main(args=None):
    rclpy.init(args=args)
    node = LowScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
