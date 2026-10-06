#!/usr/bin/env python3
"""LiDAR odometry -> ArduPilot ExternalNav (indoor GUIDED, report Q80).

    /lidar_odom/odometry  ->  /mavros/vision_pose/pose        (VISION_POSITION_ESTIMATE)
                          ->  /mavros/vision_speed/speed_twist (VISION_SPEED_ESTIMATE)

Indoors the Cube has no position, velocity or heading source (EK3 source set 1 = GPS). Source
set 2 is ExternalNav (EK3_SRC2_POSXY/VELXY/YAW = 6, VISO_TYPE = 1), selected by the operator's
RC switch (RCx_OPTION = 90). This relay feeds it lidar_odom's pose (its own drifting odometry
frame, ENU, z up) and velocity (rotated from base_footprint into that frame); the mavros plugins
convert ENU -> NED. Outdoors, with set 1 selected, ArduPilot does not fuse it but keeps the
ExternalNav frame aligned to GPS, so switching to set 2 does not jump.

lidar_odom publishes only accepted scans: a rejected scan (or a dead lidar_odom) leaves a gap,
which the Cube's EKF times out on -- with FS_EKF_ACTION = 1 it then holds.
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry


class ExtNavRelay(Node):

    def __init__(self):
        super().__init__("extnav_relay")
        self.declare_parameter("odom_topic", "/lidar_odom/odometry")
        self.declare_parameter("pose_topic", "/mavros/vision_pose/pose")
        self.declare_parameter("speed_topic", "/mavros/vision_speed/speed_twist")
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.pose_pub = self.create_publisher(PoseStamped, g("pose_topic"), 10)
        self.speed_pub = self.create_publisher(TwistStamped, g("speed_topic"), 10)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, 10)
        self.count = 0
        self.create_timer(60.0, self.stats)
        self.get_logger().info(f"extnav_relay: {g('odom_topic')} -> {g('pose_topic')} + {g('speed_topic')}")

    def on_odom(self, m):
        p = PoseStamped()
        p.header = m.header           # stamp of the scan; frame = lidar_odom's world (ENU)
        p.pose = m.pose.pose
        self.pose_pub.publish(p)
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        vx, vy = m.twist.twist.linear.x, m.twist.twist.linear.y      # base_footprint
        c, s = math.cos(yaw), math.sin(yaw)
        t = TwistStamped()
        t.header = m.header
        t.twist.linear.x = c * vx - s * vy                            # odometry frame (ENU)
        t.twist.linear.y = s * vx + c * vy
        self.speed_pub.publish(t)
        self.count += 1

    def stats(self):
        self.get_logger().info(f"extnav_relay: {self.count} poses relayed in the last 60 s")
        self.count = 0


def main(args=None):
    rclpy.init(args=args)
    node = ExtNavRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
