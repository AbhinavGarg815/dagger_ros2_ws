#!/usr/bin/env python3
"""Shadow-mode SLAM viewer: /slam/pose -> TF slam_map -> slam_base_footprint.

In shadow mode slam_toolbox publishes NOTHING on /tf -- odom already has a parent
(the GPS layer's map), and a second one would break the tree. But Foxglove only
offers display frames it has seen in TF, so /slam/map could not be drawn at all.

This puts slam_map into TF as the root of its OWN small tree, with the rover's
SLAM-estimated pose as its only child. odom is untouched. In Foxglove: display
frame slam_map, enable /slam/map, and the axes at slam_base_footprint show where
SLAM thinks the rover is.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster


class SlamPoseTf(Node):

    def __init__(self):
        super().__init__("slam_pose_tf")
        self.declare_parameter("child_frame", "slam_base_footprint")
        self.child = self.get_parameter("child_frame").value
        self.declare_parameter("map_frame", "slam_map")
        self.map_frame = self.get_parameter("map_frame").value
        self.br = TransformBroadcaster(self)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.last = None
        # slam_toolbox publishes a pose only after the rover moves 0.3 m, and
        # /slam/pose is not latched -- so republish continuously, or a parked
        # rover (or a late Foxglove) would never see the frame at all.
        self.create_subscription(PoseWithCovarianceStamped, "/slam/pose", self.on_pose, 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f"/slam/pose -> TF <map frame> -> {self.child} (viewer only)")

    def on_pose(self, m):
        self.last = m.pose.pose
        self.tick()

    def tick(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.child
        if self.last is not None:
            p = self.last
            t.transform.translation.x, t.transform.translation.y = p.position.x, p.position.y
            t.transform.rotation = p.orientation
        else:
            # Before SLAM's first pose its map is aligned with odom, so the
            # odometry pose is the right answer until the rover moves.
            try:
                o = self.buf.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            except Exception:
                return
            t.transform.translation.x = o.transform.translation.x
            t.transform.translation.y = o.transform.translation.y
            t.transform.rotation = o.transform.rotation
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SlamPoseTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
