#!/usr/bin/env python3
"""Foxglove click -> Nav2 goal / SLAM pose estimate, made safe to use.

    Foxglove 3D panel (display + fixed frame `odom` for goals; `map` for pose estimates):
      publish tool "Pose"           -> /goal_pose_ui     -> /goal_pose     (bt_navigator)
      publish tool "Pose estimate"  -> /initialpose_ui   -> /initialpose   (slam_toolbox)

Why a relay instead of letting Foxglove publish /goal_pose directly:
  - STAMP: Foxglove stamps the click time. Nav2 re-transforms the goal into the
    costmap frame (odom) at EVERY replan AT THAT STAMP, which ages out of the
    ~10 s TF buffer -> "extrapolation into the past" -> path lost -> abort
    (same failure as 2026-09-14, see tools/send_goal.py). Forwarded goals carry
    stamp 0 = "latest transform".
  - FRAME: a click in a moving frame (base_footprint, ...) with stamp 0 would be
    re-read relative to the rover every replan and never reached, so only map
    and odom goals are forwarded. Use odom for clicked goals: in GPS mode map->odom
    follows the GNSS position, and without RTK that wanders (2026-10-03: 3D fix,
    h_acc ~3 m, map->odom moved 0.68 m in 27 s parked) -- a goal clicked in map
    slid as the rover drove to it. odom is the rover's continuous frame, the one the
    planner and costmaps use. map is for lat/lon goals and SLAM.
    No TF listener on purpose: in Python it decodes all of /tf (~16 % CPU,
    measured 2026-09-29) for a conversion the documented setup never needs.
  - LOCALIZATION: in SLAM mode a goal is refused while /slam/localized is false.
    (In GPS mode nothing publishes /slam/localized and goals pass.)
Pose estimates must be in map (slam_toolbox ignores the frame id, so anything
else would land in the wrong place).

Also /plan -> /plan_viz for display: Humble's Theta* planner leaves every pose's
frame_id empty (only the Path header says odom). The controller transforms by
the header, so driving is fine, but Foxglove refuses to draw it ("Path poses
must all have the same frame_id. odom != ''"). View /plan_viz instead.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

FIXED = ("map", "odom")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GoalPoseRelay(Node):

    def __init__(self):
        super().__init__("goal_pose_relay")
        self.localized = None      # None = no SLAM (GPS mode)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/slam/localized",
                                 lambda m: setattr(self, "localized", m.data), latched)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(PoseStamped, "/goal_pose_ui", self.on_goal, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose_ui", self.on_init, 10)
        self.plan_pub = self.create_publisher(Path, "/plan_viz", 1)
        self.create_subscription(Path, "/plan", self.on_plan, 1)
        self.get_logger().info("relaying /goal_pose_ui -> /goal_pose (stamp 0) and "
                               "/initialpose_ui -> /initialpose (in map)")

    def on_goal(self, m):
        if self.localized is False:
            self.get_logger().error("goal REFUSED: SLAM is not localized (/slam/localized false)")
            return
        frame, pose = m.header.frame_id or "map", m.pose
        if frame not in FIXED:
            self.get_logger().error(f"goal REFUSED: clicked in '{frame}' -- set the Foxglove "
                                    "3D panel display frame to odom")
            return
        if frame == "map" and self.localized is None:
            self.get_logger().warn("goal in map, GPS mode: it moves with the GNSS position "
                                   "(no RTK: metres) -- set the display frame to odom")
        out = PoseStamped()
        out.header.frame_id = frame           # stamp stays 0 = latest transform
        out.pose = pose
        self.goal_pub.publish(out)
        self.get_logger().info(
            f"goal -> Nav2: {frame} ({pose.position.x:.2f}, {pose.position.y:.2f}) "
            f"yaw {math.degrees(yaw_of(pose.orientation)):.1f} deg")

    def on_plan(self, m):
        for p in m.poses:
            p.header = m.header
        self.plan_pub.publish(m)

    def on_init(self, m):
        frame, pose = m.header.frame_id or "map", m.pose.pose
        if frame != "map":
            self.get_logger().error(f"pose estimate REFUSED: set in '{frame}' -- set the Foxglove "
                                    "3D panel display frame to map")
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
