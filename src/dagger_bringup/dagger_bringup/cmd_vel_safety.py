#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class CmdVelSafety(Node):

    def __init__(self):
        super().__init__("cmd_vel_safety")

        self.declare_parameter("input_topic", "/cmd_vel_out")
        self.declare_parameter("output_topic", "/mavros/setpoint_velocity/cmd_vel_unstamped")
        self.declare_parameter("estop_topic", "/estop")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("command_timeout", 0.5)
        # The Dagger tracked chassis turns opposite to the ROS/REP-103 yaw
        # convention (commanded +angular.z / CCW makes it spin CW). Confirmed by
        # a controlled pivot sweep + visual. Negate yaw at this single FCU
        # boundary so teleop and Nav2 both turn correctly. Set false if the sign
        # is ever corrected on the ArduPilot skid-steer side.
        self.declare_parameter("invert_yaw", False)
        # Sensor freshness gate: when set, output is forced to zero unless a
        # LaserScan arrived on this topic within fresh_timeout s. Humble's
        # collision_monitor treats a stale source as "no obstacles" and passes
        # commands through, so the guard lives here at the FCU boundary. Empty = off.
        self.declare_parameter("fresh_topic", "")
        self.declare_parameter("fresh_timeout", 0.5)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.estop_topic = self.get_parameter("estop_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.invert_yaw = bool(self.get_parameter("invert_yaw").value)
        self.fresh_topic = self.get_parameter("fresh_topic").value
        self.fresh_timeout = float(self.get_parameter("fresh_timeout").value)

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.estop_active = False
        self.last_fresh_time = None
        self.stale_logged = False

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.cmd_callback, 10)
        self.create_subscription(Bool, self.estop_topic, self.estop_callback, 10)
        if self.fresh_topic:
            be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=1)
            self.create_subscription(LaserScan, self.fresh_topic, self.fresh_callback, be,
                                     raw=True)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_command)

        self.get_logger().info(
            f"cmd_vel_safety active: {self.input_topic} -> {self.output_topic} "
            f"(estop={self.estop_topic}, timeout={self.command_timeout}s, "
            f"invert_yaw={self.invert_yaw}, fresh_topic={self.fresh_topic or 'off'})"
        )

    def fresh_callback(self, _):
        self.last_fresh_time = self.get_clock().now()

    def sensor_stale(self):
        if not self.fresh_topic:
            return False
        if self.last_fresh_time is None:
            return True
        age = (self.get_clock().now() - self.last_fresh_time).nanoseconds / 1e9
        return age > self.fresh_timeout

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def estop_callback(self, msg):
        if msg.data and not self.estop_active:
            self.get_logger().warn("E-STOP engaged: forcing zero velocity")
        if not msg.data and self.estop_active:
            self.get_logger().info("E-STOP cleared")
        self.estop_active = msg.data

    def publish_command(self):
        out = Twist()

        if self.estop_active:
            self.pub.publish(out)
            return

        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        fresh_cmd = elapsed <= self.command_timeout
        if self.sensor_stale():
            if fresh_cmd and not self.stale_logged:
                self.get_logger().error(
                    f"{self.fresh_topic} stale (> {self.fresh_timeout}s): forcing zero velocity")
                self.stale_logged = True
            self.pub.publish(out)
            return
        self.stale_logged = False

        if fresh_cmd:
            # Copy into a fresh Twist (never mutate self.last_cmd, or the yaw
            # negation would flip every timer tick).
            out.linear.x = self.last_cmd.linear.x
            out.linear.y = self.last_cmd.linear.y
            out.linear.z = self.last_cmd.linear.z
            out.angular.x = self.last_cmd.angular.x
            out.angular.y = self.last_cmd.angular.y
            out.angular.z = self.last_cmd.angular.z
            if self.invert_yaw:
                out.angular.z = -out.angular.z

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
