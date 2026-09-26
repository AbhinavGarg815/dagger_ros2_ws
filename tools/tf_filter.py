#!/usr/bin/env python3
"""Relay /tf_bag -> /tf, dropping transforms whose child is in --drop.

For offline replays (tools/slam_loc_replay.sh): a recorded drive carries
gps_map_odom's map->odom, which would fight the map->odom of the SLAM being
tested. Play the bag with --remap /tf:=/tf_bag and run this.
"""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drop", nargs="+", default=["odom", "slam_base_footprint"])
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = Node("tf_filter", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    pub = node.create_publisher(TFMessage, "/tf", 100)
    drop = set(a.drop)

    def cb(m):
        keep = [t for t in m.transforms if t.child_frame_id not in drop]
        if keep:
            pub.publish(TFMessage(transforms=keep))

    node.create_subscription(TFMessage, "/tf_bag", cb, 100)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == "__main__":
    main()
