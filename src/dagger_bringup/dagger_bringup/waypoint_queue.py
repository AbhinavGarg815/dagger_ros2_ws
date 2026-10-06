#!/usr/bin/env python3
"""Waypoint routes from Foxglove clicks -> Nav2 NavigateThroughPoses.

    Foxglove 3D panel, pose tool topic /waypoint_ui  -> each click appends a waypoint
    /waypoints/path   (nav_msgs/Path, latched)        -> the queued route, drawn in Foxglove
    /waypoints/go     (std_msgs/Empty, Publish panel) -> drive the route (NavigateThroughPoses)
    /waypoints/clear  (std_msgs/Empty)                -> empty the queue

Single goals are unchanged (/goal_pose_ui -> goal_pose_relay). Waypoints must be clicked in
odom or map (as goal_pose_relay; one frame per route) and are sent with stamp 0 = latest
transform. odom routes request dagger_navigate_through_poses_odom.xml, whose RemovePassedGoals
looks the rover up in odom (the default tree uses map, for gps_goal.py routes). The queue is
kept after go, so a route can be re-run; clear starts a new one.
"""
import math
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
from std_msgs.msg import Empty

FIXED = ("odom", "map")


class WaypointQueue(Node):

    def __init__(self):
        super().__init__("waypoint_queue")
        self.odom_bt = os.path.join(get_package_share_directory("dagger_bringup"),
                                    "behavior_trees", "dagger_navigate_through_poses_odom.xml")
        self.frame = None
        self.poses = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub = self.create_publisher(Path, "/waypoints/path", latched)
        self.create_subscription(PoseStamped, "/waypoint_ui", self.on_click, 10)
        self.create_subscription(Empty, "/waypoints/go", self.on_go, 10)
        self.create_subscription(Empty, "/waypoints/clear", self.on_clear, 10)
        self.client = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        self.publish_path()
        self.get_logger().info("waypoint_queue: click on /waypoint_ui, then /waypoints/go "
                               "(/waypoints/clear to start over)")

    def publish_path(self):
        p = Path()
        p.header.frame_id = self.frame or "odom"
        p.poses = self.poses
        self.path_pub.publish(p)

    def on_click(self, m):
        frame = m.header.frame_id or "map"
        if frame not in FIXED:
            self.get_logger().error(f"waypoint REFUSED: clicked in '{frame}' -- set the Foxglove "
                                    "3D panel display frame to odom")
            return
        if self.frame and frame != self.frame:
            self.get_logger().error(f"waypoint REFUSED: this route is in {self.frame}, the click "
                                    f"in {frame} -- /waypoints/clear to start a new route")
            return
        self.frame = frame
        w = PoseStamped()
        w.header.frame_id = frame             # stamp stays 0 = latest transform
        w.pose = m.pose
        self.poses.append(w)
        q = m.pose.orientation
        self.get_logger().info(
            f"waypoint {len(self.poses)}: {frame} ({m.pose.position.x:.2f}, {m.pose.position.y:.2f}) "
            f"yaw {math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))):.0f} deg")
        self.publish_path()

    def on_clear(self, _):
        self.get_logger().info(f"route cleared ({len(self.poses)} waypoints)")
        self.poses, self.frame = [], None
        self.publish_path()

    def on_go(self, _):
        if not self.poses:
            self.get_logger().warn("no waypoints queued -- click on /waypoint_ui first")
            return
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 /navigate_through_poses not available")
            return
        goal = NavigateThroughPoses.Goal(poses=list(self.poses))
        if self.frame == "odom":
            goal.behavior_tree = self.odom_bt
        self.get_logger().info(f"route -> Nav2: {len(self.poses)} waypoints in {self.frame}")
        self.client.send_goal_async(goal).add_done_callback(self.on_accepted)

    def on_accepted(self, fut):
        h = fut.result()
        if not h.accepted:
            self.get_logger().error("route REJECTED by Nav2")
            return
        h.get_result_async().add_done_callback(
            lambda f: self.get_logger().info(f"route finished: status {f.result().status} "
                                             "(4 = succeeded, 5 = canceled, 6 = aborted)"))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointQueue()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
