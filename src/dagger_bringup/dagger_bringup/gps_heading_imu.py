#!/usr/bin/env python3
"""Dual-antenna GNSS heading -> sensor_msgs/Imu (yaw only) for the fused odom EKF.

The CUAV C-RTK 2HP moving-baseline heading (GPS_RAW_INT.yaw, centi-degrees,
compass/NED) is an absolute, drift-free vehicle heading. Fusing it into ekf_local
makes odom yaw TRUE (ENU) heading and removes gyro-integration drift (~7 deg over
3 min of pivoting on 2026-09-14).

Gated: published only when
  - the heading is valid (yaw not 0 / 65535), and
  - the rover is NOT turning (|raw gyro z| < max_yaw_rate): the GNSS heading lags
    the gyro by ~0.2-0.5 s, so during pivots it would drag odom yaw backwards.
Between samples the EKF integrates the gyro, so gating costs nothing.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavros_msgs.msg import GPSRAW
from sensor_msgs.msg import Imu


class GpsHeadingImu(Node):

    def __init__(self):
        super().__init__("gps_heading_imu")
        self.declare_parameter("gps_raw_topic", "/mavros/gpsstatus/gps1/raw")
        self.declare_parameter("gyro_topic", "/mavros/imu/data_raw")
        self.declare_parameter("output_topic", "/gps/heading_imu")
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("yaw_stddev_deg", 1.0)
        self.declare_parameter("max_yaw_rate", 0.10)  # rad/s
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.frame = g("frame_id")
        self.yaw_var = math.radians(float(g("yaw_stddev_deg"))) ** 2
        self.max_wz = float(g("max_yaw_rate"))
        self.wz = 0.0
        self.n_pub = self.n_turning = self.n_invalid = 0

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Imu, g("output_topic"), 10)
        self.create_subscription(Imu, g("gyro_topic"), self.on_gyro, be)
        self.create_subscription(GPSRAW, g("gps_raw_topic"), self.on_gps, be)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"gps_heading_imu -> {g('output_topic')} (yaw sd {g('yaw_stddev_deg')} deg, "
            f"gated |wz| < {self.max_wz} rad/s)")

    def on_gyro(self, m):
        self.wz = m.angular_velocity.z

    def on_gps(self, m):
        if m.yaw in (0, 65535):
            self.n_invalid += 1
            return
        if abs(self.wz) > self.max_wz:
            self.n_turning += 1
            return
        # GPS_RAW yaw: degrees clockwise from north -> ENU yaw: CCW from east.
        yaw = math.radians(90.0 - m.yaw / 100.0)
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
        out = Imu()
        out.header.stamp = m.header.stamp
        out.header.frame_id = self.frame
        out.orientation.z, out.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        out.orientation_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, self.yaw_var]
        out.angular_velocity_covariance[0] = -1.0     # not provided
        out.linear_acceleration_covariance[0] = -1.0  # not provided
        self.pub.publish(out)
        self.n_pub += 1

    def report(self):
        self.get_logger().info(
            f"heading published {self.n_pub}, skipped turning {self.n_turning}, "
            f"invalid {self.n_invalid} (last 10 s)")
        self.n_pub = self.n_turning = self.n_invalid = 0


def main(args=None):
    rclpy.init(args=args)
    node = GpsHeadingImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
