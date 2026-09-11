#!/usr/bin/env python3
"""Use ArduPilot's EK3 fused pose as Nav2 odometry (GPS + dual-antenna heading + IMU).

Why: KISS-ICP position JUMPS during in-place rotation (16-beam scan-match degrades),
which breaks Nav2 path-following (needs continuous position). Now that the Cube has a
dual-antenna C-RTK 2HP GNSS, ArduPilot's EK3 already fuses GPS position + velocity +
moving-baseline heading + IMU into a smooth, drift-corrected, rotation-robust local
pose, published on /mavros/local_position/odom (map->base_link). We relay that as
/odometry/filtered (odom->base_footprint) + broadcast the odom->base_footprint TF, so
Nav2 consumes it with no config change and KISS-ICP is dropped entirely.

Frames: ArduPilot local_position is a single fixed-origin (EKF home) ENU frame we
relabel "odom"; it estimates the body (base_link) which for a ground rover we treat as
base_footprint (z flattened). Later a map->odom layer (mrg_slam / navsat) can sit on
top; for now this single GPS-anchored frame is the Nav2 world.

Stream rate: /mavros/local_position/odom defaults to 2 Hz (SR0_POSITION); raise
LOCAL_POSITION_NED (msg 32) to ~20 Hz via MAV_CMD_SET_MESSAGE_INTERVAL (not persistent
-- request on each bring-up).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class FcuOdomRelay(Node):

    def __init__(self):
        super().__init__("fcu_odom_relay")
        self.declare_parameter("input_topic", "/mavros/local_position/odom")
        self.declare_parameter("output_topic", "/odometry/filtered")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("flatten_z", True)   # 2D ground rover
        self.declare_parameter("publish_tf", True)

        g = self.get_parameter
        self.odom_frame = g("odom_frame").value
        self.base_frame = g("base_frame").value
        self.flatten_z = bool(g("flatten_z").value)
        self.publish_tf = bool(g("publish_tf").value)

        # mavros publishes local_position with BEST_EFFORT sensor QoS.
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Odometry, g("output_topic").value, rel)
        self.br = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Odometry, g("input_topic").value, self.cb, be)
        self.get_logger().info(
            f"fcu_odom_relay: {g('input_topic').value} -> {g('output_topic').value} "
            f"({self.odom_frame}->{self.base_frame}), publish_tf={self.publish_tf}")

    def cb(self, msg):
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose = msg.pose
        out.twist = msg.twist
        if self.flatten_z:
            out.pose.pose.position.z = 0.0
        self.pub.publish(out)

        if self.br is not None:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = out.pose.pose.position.x
            t.transform.translation.y = out.pose.pose.position.y
            t.transform.translation.z = 0.0 if self.flatten_z else out.pose.pose.position.z
            t.transform.rotation = out.pose.pose.orientation
            self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FcuOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
