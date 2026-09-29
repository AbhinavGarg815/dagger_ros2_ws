#!/usr/bin/env python3
"""Send GPS (lat/lon) waypoints to Nav2 in the `map` frame.

    python3 tools/gps_goal.py --mission --dry-run      # FCU mission -> map (x, y), no motion
    python3 tools/gps_goal.py --mission                # follow the same mission Mission Planner flies
    python3 tools/gps_goal.py --ll "26.74798,75.85534 26.74801,75.85540"
    python3 tools/gps_goal.py --enu "8,0 8,5"          # metres East,North of the current position

Conversion needs no datum: EK3 publishes its global (/mavros/global_position/global)
and local (/mavros/local_position/odom) estimates of the SAME pose, so
    goal_map = local_now + ENU(goal_latlon - global_now)
(WGS-84 local tangent plane; cm-accurate within ~1 km). That is GPS mode
(nav2_gps.launch.py, map->odom from gps_map_odom; map = ArduPilot EK3 local ENU).

SLAM mode (T2 map_source:=slam, T5 slam.launch.py mode:=localization; map = the
saved SLAM map) goes through the map's <name>_geo.yaml instead. Detected
automatically (--map-source), two anchors for lat/lon:
  --anchor geo  (default) the same lat/lon always lands on the same map point;
                error = the map's georeference (~1 m + that day's GNSS bias)
  --anchor live rover's SLAM pose + (goal - current GNSS fix); error = TODAY's
                GNSS bias (2-5 m seen without RTK, Q43)
--enu in SLAM mode is relative to the rover's SLAM pose: no GNSS position error.
 Global costmap is a 40 m rolling window -> each
waypoint should be < ~18 m from the rover. MOTION COMMAND -- the user runs it;
Ctrl-C cancels the goal.
"""
import argparse
import math
import sys

import os

import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
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
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
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

    def map_source(self):
        end = self.get_clock().now().nanoseconds * 1e-9 + 3.0
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        names = set(self.get_node_names())
        if "gps_map_odom" in names:
            return "gps"
        if "slam_geo_init" in names:
            return "slam"
        return None

    def geo_file(self):
        cli = self.create_client(GetParameters, "/slam_geo_init/get_parameters")
        if cli.wait_for_service(timeout_sec=3.0):
            fut = cli.call_async(GetParameters.Request(names=["geo_file"]))
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
            r = fut.result()
            if r and r.values and r.values[0].string_value:
                return r.values[0].string_value
        return None

    def map_pose(self, timeout=5.0):
        end = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.tf.lookup_transform("map", "base_footprint", Time())
            except tf2_ros.TransformException:
                continue
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        return None

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
    ap.add_argument("--map-source", choices=["auto", "gps", "slam"], default="auto",
                    help="who owns map->odom (auto: gps_map_odom running -> gps, slam_geo_init -> slam)")
    ap.add_argument("--anchor", choices=["geo", "live"], default="geo",
                    help="SLAM mode, lat/lon goals: fixed georeference or offset from the live GNSS fix")
    ap.add_argument("--geo", help="SLAM mode: <map>_geo.yaml (default: slam_geo_init's geo_file)")
    a = ap.parse_args()

    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    n = GpsGoal()
    if not n.wait(["fix", "local"]):
        sys.exit("no /mavros/global_position/global or /mavros/local_position/odom")
    lat0, lon0 = n.fix.latitude, n.fix.longitude
    source = n.map_source() if a.map_source == "auto" else a.map_source
    if source is None:
        sys.exit("cannot tell who owns map->odom: neither gps_map_odom nor slam_geo_init is running")
    geo = None
    if source == "slam":
        try:
            from dagger_bringup.geo_map import GeoMap
        except ImportError:   # workspace not sourced: use the source tree
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                            "src", "dagger_bringup"))
            from dagger_bringup.geo_map import GeoMap
        path = a.geo or n.geo_file()
        if not path:
            sys.exit("SLAM mode: no --geo and slam_geo_init did not report its geo_file")
        geo = GeoMap(path)
        mp = n.map_pose()
        if mp is None:
            sys.exit("SLAM mode: no map->base_footprint transform (is T5 localized?)")
        x0, y0 = mp[0], mp[1]
        print(f"SLAM map mode: {os.path.basename(path)} (georef residual {geo.residual_rms} m), "
              f"lat/lon anchor '{a.anchor}'")
    else:
        x0, y0 = n.local.pose.pose.position.x, n.local.pose.pose.position.y
        print("GPS map mode (ArduPilot local frame)")

    def ll_to_map(la, lo):
        if geo is None:
            e, nn = ll_to_en(la, lo, lat0, lon0)
            return x0 + e, y0 + nn
        if a.anchor == "geo":
            return geo.to_map(la, lo)[:2]
        dx, dy = geo.enu_to_map_vec(*ll_to_en(la, lo, lat0, lon0))
        return x0 + dx, y0 + dy

    def enu_to_map(e, nn):
        if geo is None:
            return x0 + e, y0 + nn
        dx, dy = geo.enu_to_map_vec(e, nn)
        return x0 + dx, y0 + dy

    if a.mission:
        if not n.wait(["mission"]):
            sys.exit("no /mavros/mission/waypoints (is a mission loaded on the FCU?)")
        wps = [(w.x_lat, w.y_long) for i, w in enumerate(n.mission.waypoints)
               if i > 0 and w.command == MAV_CMD_NAV_WAYPOINT and (w.x_lat or w.y_long)]
        if not wps:
            sys.exit("FCU mission has no NAV_WAYPOINT items with a position")
        xy = [ll_to_map(la, lo) for la, lo in wps]
    elif a.ll:
        wps = [tuple(map(float, p.split(","))) for p in a.ll.split()]
        xy = [ll_to_map(la, lo) for la, lo in wps]
    else:
        xy = [enu_to_map(*map(float, p.split(","))) for p in a.enu.split()]
        wps = [None] * len(xy)

    print(f"rover: lat {lat0:.7f} lon {lon0:.7f} | map ({x0:.2f}, {y0:.2f})")
    pts, prev = [], (x0, y0)
    for i, ((x, y), ll) in enumerate(zip(xy, wps)):
        dist = math.hypot(x - x0, y - y0)
        # compass bearing from the rover, in ENU (map axes rotated back in SLAM mode)
        e, nn = (x - x0, y - y0)
        if geo is not None:
            c, sn = math.cos(geo.rot), math.sin(geo.rot)
            e, nn = c * e - sn * nn, sn * e + c * nn
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
    if geo is None:
        lx, ly = n.local.pose.pose.position.x, n.local.pose.pose.position.y
    else:
        lx, ly = (n.map_pose() or (float("nan"), float("nan")))[:2]
    print(f"RESULT {name} | map ({lx:.2f}, {ly:.2f}) | map error to final wp "
          f"{math.hypot(lx - pts[-1][0], ly - pts[-1][1]):.2f} m {last}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
