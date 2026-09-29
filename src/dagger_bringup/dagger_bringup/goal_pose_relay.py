#!/usr/bin/env python3
"""Foxglove click -> Nav2 goal / SLAM pose estimate, made safe to use.

    Foxglove 3D panel (display frame `map`):
      publish tool "Pose"           -> /goal_pose_ui     -> /goal_pose     (bt_navigator)
      publish tool "Pose estimate"  -> /initialpose_ui   -> /initialpose   (slam_toolbox)

Why a relay instead of letting Foxglove publish /goal_pose directly:
  - STAMP: Foxglove stamps the click time. Nav2 re-transforms the goal into the
    costmap frame (odom) at EVERY replan AT THAT STAMP, which ages out of the
    ~10 s TF buffer -> "extrapolation into the past" -> path lost -> abort
    (same failure as 2026-09-14, see tools/send_goal.py). Forwarded goals carry
    stamp 0 = "latest transform".
  - FRAME: a click in a moving frame (base_footprint, ...) with stamp 0 would be
    re-read relative to the rover every replan and never reached. Anything that
    is not map/odom is converted into map at receipt.
  - LOCALIZATION: in SLAM mode a goal is refused while /slam/localized is false.
    (In GPS mode nothing publishes /slam/localized and goals pass.)
Pose estimates are always converted into map (slam_toolbox ignores the frame id).
"""
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_pose

FIXED = ("map", "odom")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GoalPoseRelay(Node):

    def __init__(self):
        super().__init__("goal_pose_relay")
        self.tf = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        tf2_ros.TransformListener(self.tf, self)
        self.localized = None      # None = no SLAM (GPS mode)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/slam/localized",
                                 lambda m: setattr(self, "localized", m.data), latched)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(PoseStamped, "/goal_pose_ui", self.on_goal, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose_ui", self.on_init, 10)
        self.get_logger().info("relaying /goal_pose_ui -> /goal_pose (stamp 0) and "
                               "/initialpose_ui -> /initialpose (in map)")

    def to_frame(self, pose, frame, target):
        """Pose in `frame` -> `target` with the latest transform, or None."""
        try:
            t = self.tf.lookup_transform(target, frame, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"cannot transform {frame} -> {target}: {e}")
            return None
        return do_transform_pose(pose, t)

    def on_goal(self, m):
        if self.localized is False:
            self.get_logger().error("goal REFUSED: SLAM is not localized (/slam/localized false)")
            return
        frame, pose = m.header.frame_id or "map", m.pose
        if frame not in FIXED:
            pose = self.to_frame(pose, frame, "map")
            if pose is None:
                self.get_logger().error("goal REFUSED: set the Foxglove display frame to map")
                return
            frame = "map"
        out = PoseStamped()
        out.header.frame_id = frame           # stamp stays 0 = latest transform
        out.pose = pose
        self.goal_pub.publish(out)
        self.get_logger().info(
            f"goal -> Nav2: {frame} ({pose.position.x:.2f}, {pose.position.y:.2f}) "
            f"yaw {math.degrees(yaw_of(pose.orientation)):.1f} deg"
            + ("" if m.header.frame_id in FIXED else f" [clicked in {m.header.frame_id}]"))

    def on_init(self, m):
        frame, pose = m.header.frame_id or "map", m.pose.pose
        if frame != "map":
            pose = self.to_frame(pose, frame, "map")
            if pose is None:
                return
        out = PoseWithCovarianceStamped()
        out.header.frame_id = "map"
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.pose = pose
        out.pose.covariance = m.pose.covariance
        self.init_pub.publish(out)
        self.get_logger().info(
            f"pose estimate -> SLAM: map ({pose.position.x:.2f}, {pose.position.y:.2f}) "
            f"yaw {math.degrees(yaw_of(pose.orientation)):.1f} deg")


def main():
    rclpy.init()
    node = GoalPoseRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
