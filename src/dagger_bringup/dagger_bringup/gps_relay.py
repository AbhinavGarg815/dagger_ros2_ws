#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix


class GpsRelay(Node):

    def __init__(self):
        super().__init__("gps_relay")

        self.declare_parameter("input_topic", "/mavros/global_position/raw/fix")
        self.declare_parameter("output_topic", "/dagger/gps/navsat")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(NavSatFix, self.output_topic, 10)
        self.create_subscription(
            NavSatFix, self.input_topic, self.callback, qos_profile_sensor_data
        )

        self.count = 0
        self.get_logger().info(
            f"gps_relay: {self.input_topic} -> {self.output_topic}"
        )

    def callback(self, msg):
        self.pub.publish(msg)
        self.count += 1
        if self.count % 50 == 1:
            self.get_logger().info(
                f"relayed GPS fix #{self.count} (status={msg.status.status})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = GpsRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
