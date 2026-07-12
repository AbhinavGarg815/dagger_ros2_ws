#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2

import zmq


class LidarSender(Node):

    def __init__(self):
        super().__init__("lidar_sender")

        context = zmq.Context()

        self.socket = context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")

        # Optional: keep only latest frame
        self.socket.setsockopt(zmq.SNDHWM, 1)

        self.subscription = self.create_subscription(
            PointCloud2,
            "/rslidar_points",
            self.pointcloud_callback,
            10
        )

        self.get_logger().info("Waiting for /rslidar_points...")

    def pointcloud_callback(self, msg):

        metadata = {
            "stamp_sec": msg.header.stamp.sec,
            "stamp_nanosec": msg.header.stamp.nanosec,
            "frame_id": msg.header.frame_id,

            "height": msg.height,
            "width": msg.width,

            "is_bigendian": msg.is_bigendian,
            "point_step": msg.point_step,
            "row_step": msg.row_step,
            "is_dense": msg.is_dense,

            "fields": [
                {
                    "name": f.name,
                    "offset": f.offset,
                    "datatype": f.datatype,
                    "count": f.count
                }
                for f in msg.fields
            ]
        }

        self.socket.send_multipart([
            json.dumps(metadata).encode(),
            bytes(msg.data)
        ])

        self.get_logger().info(
            f"Sent cloud ({msg.width} points)"
        )


def main(args=None):

    rclpy.init(args=args)

    node = LidarSender()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
