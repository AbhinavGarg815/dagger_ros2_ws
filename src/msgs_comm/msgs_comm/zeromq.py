#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import zmq


class ZMQPublisher(Node):

    def __init__(self):
        super().__init__("zmq_publisher")

        context = zmq.Context()

        self.socket = context.socket(zmq.PUB)

        # Listen on port 5555
        self.socket.bind("tcp://*:5555")

        self.count = 0

        self.timer = self.create_timer(
            1.0,
            self.publish_message
        )

        self.get_logger().info("ZeroMQ Publisher Started")

    def publish_message(self):

        msg = f"Hello {self.count}"

        self.socket.send_string(msg)

        self.get_logger().info(f"Sent: {msg}")

        self.count += 1


def main(args=None):

    rclpy.init(args=args)

    node = ZMQPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
