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
import math

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
        # mavros publishes local_position/odom with ALL-ZERO covariance, which
        # robot_localization would treat as a perfect measurement. When set (>0),
        # these variances are written into the twist covariance diagonal
        # [vx, vy, vyaw] so the relay can feed an EKF (odom_source:=fused).
        self.declare_parameter("twist_variance", [0.0, 0.0, 0.0])
        # Zero-velocity deadband (m/s): EK3 reports ~5 mm/s bias + 5 mm/s noise at
        # rest (GPS Doppler), which an EKF integrates into ~0.3 m/min of creep.
        # |v| below this -> published as exactly 0. 0 = off. Drive speeds are >=0.1.
        self.declare_parameter("zero_velocity_deadband", 0.0)
        # Pivot gate (rad/s): while the rover spins in place, EK3's body velocity is
        # dominated by the GNSS-antenna lever arm (the antenna circles the pivot
        # centre) -> up to 0.9 m of FAKE translation per 180 deg spin. That smears
        # every obstacle already in the costmap, the planner then routes backwards
        # out of the "boxed in" pose, the shim pivots again -> runaway (2026-09-16,
        # bag pf_lane_radius_fix: 633 deg of spin, odom wandered ~1 m while parked).
        # A tracked vehicle pivoting on the spot translates ~0, so above this yaw
        # rate we publish zero linear velocity and let the gyro carry the heading.
        self.declare_parameter("pivot_yaw_rate_gate", 0.0)
        # Non-holonomic constraint: a tracked rover cannot move sideways, but EK3's body
        # vy is not ~0 -- it read up to -0.17 m/s against +0.22 m/s forward (an apparent
        # 37 deg crab) while accelerating from rest with the gyro at 0 (2026-09-17, bag
        # pf_foxglove_turnaround_2). The fused odom slid 1 m right of the line and Nav2
        # steered a real left curve to "fix" it. True = publish vy as exactly 0, so the
        # EKF receives vy=0 as a measurement (a hard lateral constraint).
        self.declare_parameter("zero_lateral_velocity", False)

        g = self.get_parameter
        self.odom_frame = g("odom_frame").value
        self.base_frame = g("base_frame").value
        self.flatten_z = bool(g("flatten_z").value)
        self.publish_tf = bool(g("publish_tf").value)
        self.twist_var = [float(v) for v in g("twist_variance").value]
        self.zv_deadband = float(g("zero_velocity_deadband").value)
        self.pivot_gate = float(g("pivot_yaw_rate_gate").value)
        self.zero_lateral = bool(g("zero_lateral_velocity").value)

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
        lin = out.twist.twist.linear
        if self.zv_deadband > 0.0 and math.hypot(lin.x, lin.y) < self.zv_deadband:
            lin.x = lin.y = 0.0
        if self.pivot_gate > 0.0 and abs(out.twist.twist.angular.z) > self.pivot_gate:
            lin.x = lin.y = 0.0
        if self.zero_lateral:
            lin.y = 0.0
        if any(self.twist_var):
            cov = list(out.twist.covariance)
            for idx, var in zip((0, 7, 35), self.twist_var):  # vx, vy, vyaw
                cov[idx] = var
            out.twist.covariance = cov
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
