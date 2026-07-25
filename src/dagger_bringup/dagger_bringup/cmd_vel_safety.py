#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
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

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.estop_topic = self.get_parameter("estop_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.invert_yaw = bool(self.get_parameter("invert_yaw").value)

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.estop_active = False

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.cmd_callback, 10)
        self.create_subscription(Bool, self.estop_topic, self.estop_callback, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_command)

        self.get_logger().info(
            f"cmd_vel_safety active: {self.input_topic} -> {self.output_topic} "
            f"(estop={self.estop_topic}, timeout={self.command_timeout}s, "
            f"invert_yaw={self.invert_yaw})"
        )

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
        if elapsed <= self.command_timeout:
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
