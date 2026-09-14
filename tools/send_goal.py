#!/usr/bin/env python3
"""Send a Nav2 goal RELATIVE TO THE LIVE POSE (odom frame). Ctrl-C CANCELS the goal.

    python3 tools/send_goal.py 5              # 5 m straight ahead, keep heading
    python3 tools/send_goal.py 5 --left 1     # 5 m ahead, 1 m left
    python3 tools/send_goal.py 0 --dyaw 90    # pure heading change (prefer /spin for this)
    python3 tools/send_goal.py --wp "5,0 10,0 10,4"   # waypoints (fwd,left in START body frame)

Goals are recomputed from /odometry/filtered every call, so odom resets on relaunch
don't matter. MOTION COMMAND -- run it yourself with a hand on the E-stop.
"""
import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses


def q_yaw(y):
    return 0.0, 0.0, math.sin(y / 2), math.cos(y / 2)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Sender(Node):
    def __init__(self, frame):
        super().__init__("dagger_send_goal")
        self.frame = frame
        self.pose = None
        self.create_subscription(Odometry, "/odometry/filtered", self._od,
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _od(self, m):
        self.pose = m.pose.pose

    def ps(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        (p.pose.orientation.x, p.pose.orientation.y,
         p.pose.orientation.z, p.pose.orientation.w) = q_yaw(yaw)
        return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fwd", type=float, nargs="?", default=0.0, help="metres ahead")
    ap.add_argument("--left", type=float, default=0.0, help="metres left")
    ap.add_argument("--dyaw", type=float, default=None,
                    help="final heading change, deg (default: face along the segment)")
    ap.add_argument("--wp", default=None, help='waypoints "f,l f,l ..." in start body frame')
    ap.add_argument("--to", default=None,
                    help='absolute goal "x,y[,yaw_deg]" in --frame (e.g. return to start)')
    ap.add_argument("--frame", default="odom")
    a = ap.parse_args()

    # Plain Python SIGINT -> KeyboardInterrupt, so the context stays alive long
    # enough to send the cancel (rclpy's default handler shuts it down first).
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    n = Sender(a.frame)
    while rclpy.ok() and n.pose is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    x0, y0 = n.pose.position.x, n.pose.position.y
    th0 = yaw_of(n.pose.orientation)
    c, s = math.cos(th0), math.sin(th0)

    def body_to_odom(f, l):
        return x0 + c * f - s * l, y0 + s * f + c * l

    pts = ([tuple(map(float, p.split(","))) for p in a.wp.split()] if a.wp
           else [(a.fwd, a.left)])
    poses, prev = [], (0.0, 0.0)
    for i, (f, l) in enumerate(pts):
        seg = math.atan2(l - prev[1], f - prev[0]) if (f, l) != prev else 0.0
        yaw = th0 + seg
        if i == len(pts) - 1 and a.dyaw is not None:
            yaw = th0 + math.radians(a.dyaw)
        poses.append(n.ps(*body_to_odom(f, l), yaw))
        prev = (f, l)

    if a.to:
        v = [float(s) for s in a.to.split(",")]
        yaw = math.radians(v[2]) if len(v) > 2 else math.atan2(v[1] - y0, v[0] - x0)
        poses = [n.ps(v[0], v[1], yaw)]

    print(f"start pose ({x0:.2f}, {y0:.2f}) yaw {math.degrees(th0):.1f} deg [{a.frame}]")
    for p in poses:
        print(f"  goal ({p.pose.position.x:.2f}, {p.pose.position.y:.2f}) "
              f"yaw {math.degrees(yaw_of(p.pose.orientation)):.1f} deg")

    if len(poses) == 1:
        cli = ActionClient(n, NavigateToPose, "/navigate_to_pose")
        goal = NavigateToPose.Goal(pose=poses[0])
    else:
        cli = ActionClient(n, NavigateThroughPoses, "/navigate_through_poses")
        goal = NavigateThroughPoses.Goal(poses=poses)
    if not cli.wait_for_server(timeout_sec=5.0):
        sys.exit("Nav2 action server not available")

    last = [0.0]

    def fb(msg):
        f = msg.feedback
        now = n.get_clock().now().nanoseconds * 1e-9
        if now - last[0] >= 1.0:
            last[0] = now
            extra = f" remaining_poses {f.number_of_poses_remaining}" if hasattr(
                f, "number_of_poses_remaining") else ""
            print(f"  dist_remaining {f.distance_remaining:6.2f} m  recoveries "
                  f"{f.number_of_recoveries}  t {f.navigation_time.sec}s{extra}")

    fut = cli.send_goal_async(goal, feedback_callback=fb)
    rclpy.spin_until_future_complete(n, fut)
    gh = fut.result()
    if not gh.accepted:
        sys.exit("goal REJECTED")
    print("goal accepted -- Ctrl-C cancels")
    res = gh.get_result_async()
    try:
        rclpy.spin_until_future_complete(n, res)
    except KeyboardInterrupt:
        print("\nCANCELLING goal ...")
        cf = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(n, cf, timeout_sec=3.0)
        print("cancel sent")
        return
    st = res.result().status
    name = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED"}.get(st, str(st))
    p = n.pose
    print(f"RESULT {name} | final ({p.position.x:.2f}, {p.position.y:.2f}) "
          f"yaw {math.degrees(yaw_of(p.orientation)):.1f} deg | goal err "
          f"{math.hypot(p.position.x - poses[-1].pose.position.x, p.position.y - poses[-1].pose.position.y):.2f} m")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
