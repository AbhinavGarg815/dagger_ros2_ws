#!/usr/bin/env python3
"""Send GPS (lat/lon) waypoints to Nav2 in the `map` frame (= ArduPilot EK3 local ENU).

    python3 tools/gps_goal.py --mission --dry-run      # FCU mission -> map (x, y), no motion
    python3 tools/gps_goal.py --mission                # follow the same mission Mission Planner flies
    python3 tools/gps_goal.py --ll "26.74798,75.85534 26.74801,75.85540"
    python3 tools/gps_goal.py --enu "8,0 8,5"          # metres East,North of the current position

Conversion needs no datum: EK3 publishes its global (/mavros/global_position/global)
and local (/mavros/local_position/odom) estimates of the SAME pose, so
    goal_map = local_now + ENU(goal_latlon - global_now)
(WGS-84 local tangent plane; cm-accurate within ~1 km). Requires nav2_gps.launch.py
(map->odom from gps_map_odom). Global costmap is a 40 m rolling window -> each
waypoint should be < ~18 m from the rover. MOTION COMMAND -- the user runs it;
Ctrl-C cancels the goal.
"""
import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import WaypointList
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from sensor_msgs.msg import NavSatFix

A, E2 = 6378137.0, 6.69437999014e-3  # WGS-84
MAV_CMD_NAV_WAYPOINT = 16


def ll_to_en(lat, lon, lat0, lon0):
    """East/North metres of (lat, lon) relative to (lat0, lon0)."""
    p = math.radians(lat0)
    s = 1 - E2 * math.sin(p) ** 2
    m = A * (1 - E2) / s ** 1.5          # meridian radius
    n = A / math.sqrt(s)                 # prime-vertical radius
    return math.radians(lon - lon0) * n * math.cos(p), math.radians(lat - lat0) * m


class GpsGoal(Node):
    def __init__(self):
        super().__init__("dagger_gps_goal")
        self.fix = self.local = self.mission = None
        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 lambda m: setattr(self, "fix", m), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/mavros/local_position/odom",
                                 lambda m: setattr(self, "local", m), qos_profile_sensor_data)
        self.create_subscription(
            WaypointList, "/mavros/mission/waypoints", lambda m: setattr(self, "mission", m),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def wait(self, attrs, timeout=5.0):
        end = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while rclpy.ok() and any(getattr(self, a) is None for a in attrs):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds * 1e-9 > end:
                return False
        return True

    def pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = "map"
        # Stamp 0 = latest transform (a send-time stamp ages out of the TF buffer
        # while BT nodes keep re-transforming the goals -> abort; see send_goal.py).
        p.header.stamp.sec, p.header.stamp.nanosec = 0, 0
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--mission", action="store_true", help="NAV_WAYPOINTs of the FCU mission")
    src.add_argument("--ll", help='"lat,lon lat,lon ..."')
    src.add_argument("--enu", help='"east,north ..." metres from the current position')
    ap.add_argument("--dry-run", action="store_true", help="print map coordinates, send nothing")
    ap.add_argument("--max-dist", type=float, default=18.0,
                    help="refuse waypoints farther than this from the rover (m)")
    a = ap.parse_args()

    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    n = GpsGoal()
    if not n.wait(["fix", "local"]):
        sys.exit("no /mavros/global_position/global or /mavros/local_position/odom")
    lat0, lon0 = n.fix.latitude, n.fix.longitude
    x0, y0 = n.local.pose.pose.position.x, n.local.pose.pose.position.y

    if a.mission:
        if not n.wait(["mission"]):
            sys.exit("no /mavros/mission/waypoints (is a mission loaded on the FCU?)")
        wps = [(w.x_lat, w.y_long) for i, w in enumerate(n.mission.waypoints)
               if i > 0 and w.command == MAV_CMD_NAV_WAYPOINT and (w.x_lat or w.y_long)]
        if not wps:
            sys.exit("FCU mission has no NAV_WAYPOINT items with a position")
        en = [ll_to_en(la, lo, lat0, lon0) for la, lo in wps]
    elif a.ll:
        wps = [tuple(map(float, p.split(","))) for p in a.ll.split()]
        en = [ll_to_en(la, lo, lat0, lon0) for la, lo in wps]
    else:
        en = [tuple(map(float, p.split(","))) for p in a.enu.split()]
        wps = [None] * len(en)

    print(f"rover: lat {lat0:.7f} lon {lon0:.7f} | map ({x0:.2f}, {y0:.2f})")
    pts, prev = [], (x0, y0)
    for i, ((e, nn), ll) in enumerate(zip(en, wps)):
        x, y = x0 + e, y0 + nn
        dist = math.hypot(x - x0, y - y0)
        yaw = math.atan2(y - prev[1], x - prev[0])
        pts.append((x, y, yaw))
        tag = f"lat {ll[0]:.7f} lon {ll[1]:.7f} -> " if ll else ""
        print(f"  wp{i + 1}: {tag}map ({x:.2f}, {y:.2f})  {dist:.1f} m from rover, "
              f"bearing {math.degrees(math.atan2(e, nn)) % 360:.0f} deg (compass)")
        if dist > a.max_dist:
            sys.exit(f"wp{i + 1} is {dist:.1f} m away (> --max-dist {a.max_dist}); the 40 m rolling "
                     "global costmap cannot plan that far -- split the route")
        prev = (x, y)
    if a.dry_run:
        return

    poses = [n.pose(*p) for p in pts]
    if len(poses) == 1:
        cli, goal = ActionClient(n, NavigateToPose, "/navigate_to_pose"), NavigateToPose.Goal(pose=poses[0])
    else:
        cli = ActionClient(n, NavigateThroughPoses, "/navigate_through_poses")
        goal = NavigateThroughPoses.Goal(poses=poses)
    if not cli.wait_for_server(timeout_sec=5.0):
        sys.exit("Nav2 action server not available")
    fut = cli.send_goal_async(goal)
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
        rclpy.spin_until_future_complete(n, gh.cancel_goal_async(), timeout_sec=3.0)
        return
    st = res.result().status
    name = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED"}.get(st, str(st))
    rclpy.spin_once(n, timeout_sec=0.5)
    fx = n.fix
    e, nn = ll_to_en(fx.latitude, fx.longitude, *(wps[-1] if wps[-1] else (lat0, lon0)))
    last = f"GPS distance to final wp {math.hypot(e, nn):.2f} m" if wps[-1] else ""
    lp = n.local.pose.pose.position
    print(f"RESULT {name} | map ({lp.x:.2f}, {lp.y:.2f}) | map error to final wp "
          f"{math.hypot(lp.x - pts[-1][0], lp.y - pts[-1][1]):.2f} m {last}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
