#!/usr/bin/env python3
"""Feed our drift-free yaw to ArduPilot as ExternalNav yaw (no RC override, no compass).

Why: the Cube is compassless (EK3_SRC1_YAW was GSF), so ArduPilot's own yaw is only
observable while translating (GPS velocity). At rest / during in-place rotation the
GSF yaw + gyro-bias estimate drift -> GUIDED self-rotates and in-place turns fail.
Our robot_localization EKF fuses the CLEAN gyro (/mavros/imu/data_raw) so
/odometry/filtered yaw is drift-free. We relay that yaw to the FCU as
VISION_POSITION_ESTIMATE (via mavros /mavros/vision_pose/pose) and set
EK3_SRC1_YAW=6 (ExternalNav). ArduPilot then steers on OUR heading -> GUIDED works
at rest and in-place, over pure MAVLink (no RC override).

Frame alignment (the crux): our odom yaw has an arbitrary offset from true north,
but ArduPilot fuses GPS velocity (earth frame) alongside this yaw -- a constant
offset would look like permanent sideslip and get rejected. So we earth-align:
while the rover drives straight (GSF yaw is valid when moving), we sample
    offset = wrap(gps_course_over_ground - our_odom_yaw)
and, once stable, apply out_yaw = wrap(our_odom_yaw + offset). The drift-free gyro
then holds that alignment through in-place turns. Everything is ENU here; the mavros
vision_pose plugin does ENU->NED for the FCU.

Until aligned (or a manual offset is given) we publish NOTHING, so ArduPilot keeps
using its GSF fallback (EK3_GSF_USE_MASK) -- safe hand-off, never a wrong yaw.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ExtNavYawRelay(Node):

    def __init__(self):
        super().__init__("extnav_yaw_relay")

        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("gps_vel_topic", "/mavros/global_position/raw/gps_vel")
        self.declare_parameter("output_topic", "/mavros/vision_pose/pose")
        self.declare_parameter("publish_rate", 15.0)
        # Auto earth-alignment (option A) tuning:
        self.declare_parameter("auto_align", True)
        self.declare_parameter("min_align_speed", 0.5)      # m/s straight-drive gate
        self.declare_parameter("max_align_yaw_rate", 0.10)  # rad/s (must be ~straight)
        self.declare_parameter("align_samples", 25)         # consistent samples to lock
        self.declare_parameter("align_tol_deg", 8.0)        # sample spread to accept
        # Manual override: if set (not NaN), skip auto-align and use this offset.
        self.declare_parameter("manual_offset_deg", float("nan"))
        # Send our odom position too (ignored by FCU while POSXY=GPS) or zeros.
        self.declare_parameter("send_position", True)

        g = self.get_parameter
        self.odom_topic = g("odom_topic").value
        self.gps_vel_topic = g("gps_vel_topic").value
        self.output_topic = g("output_topic").value
        self.publish_rate = float(g("publish_rate").value)
        self.auto_align = bool(g("auto_align").value)
        self.min_align_speed = float(g("min_align_speed").value)
        self.max_align_yaw_rate = float(g("max_align_yaw_rate").value)
        self.align_samples = int(g("align_samples").value)
        self.align_tol = math.radians(float(g("align_tol_deg").value))
        self.send_position = bool(g("send_position").value)

        manual = float(g("manual_offset_deg").value)
        self.offset = None
        self.aligned = False
        if not math.isnan(manual):
            self.offset = math.radians(manual)
            self.aligned = True
            self.get_logger().info(f"extnav_yaw_relay: manual offset {manual:.1f} deg -> aligned")

        self.last_odom = None      # (pose, yaw)
        self.gps_course = None
        self.gps_speed = 0.0
        self._samples = []

        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(PoseStamped, self.output_topic, rel)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        self.create_subscription(TwistStamped, self.gps_vel_topic,
                                 self.gps_vel_cb, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.tick)

        self.get_logger().info(
            f"extnav_yaw_relay: {self.odom_topic} yaw -> {self.output_topic} "
            f"(auto_align={self.auto_align}, gate v>{self.min_align_speed} m/s "
            f"|w|<{self.max_align_yaw_rate} rad/s, need {self.align_samples} samples)")

    def odom_cb(self, msg):
        self.last_odom = (msg.pose.pose, yaw_from_quat(msg.pose.pose.orientation),
                          msg.twist.twist.angular.z)

    def gps_vel_cb(self, msg):
        vx = msg.twist.linear.x  # ENU east
        vy = msg.twist.linear.y  # ENU north
        self.gps_speed = math.hypot(vx, vy)
        if self.gps_speed >= self.min_align_speed:
            self.gps_course = math.atan2(vy, vx)

    def _try_align(self):
        if self.last_odom is None or self.gps_course is None:
            return
        _, our_yaw, our_wz = self.last_odom
        if self.gps_speed < self.min_align_speed or abs(our_wz) > self.max_align_yaw_rate:
            return  # not a clean straight-drive sample
        self._samples.append(wrap(self.gps_course - our_yaw))
        if len(self._samples) < self.align_samples:
            return
        # accept only if the recent samples agree (circular spread small)
        window = self._samples[-self.align_samples:]
        mean = math.atan2(sum(math.sin(s) for s in window),
                          sum(math.cos(s) for s in window))
        spread = max(abs(wrap(s - mean)) for s in window)
        if spread <= self.align_tol:
            self.offset = mean
            self.aligned = True
            self.get_logger().info(
                f"extnav_yaw_relay: EARTH-ALIGNED offset={math.degrees(mean):.1f} deg "
                f"(spread {math.degrees(spread):.1f} deg) -> publishing ExtNav yaw")
        else:
            self._samples = self._samples[-self.align_samples // 2:]  # keep trying

    def tick(self):
        if not self.aligned:
            if self.auto_align:
                self._try_align()
            return
        if self.last_odom is None:
            return
        pose, our_yaw, _ = self.last_odom
        out_yaw = wrap(our_yaw + self.offset)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        if self.send_position:
            msg.pose.position.x = pose.position.x
            msg.pose.position.y = pose.position.y
            msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(out_yaw / 2.0)
        msg.pose.orientation.w = math.cos(out_yaw / 2.0)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExtNavYawRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
