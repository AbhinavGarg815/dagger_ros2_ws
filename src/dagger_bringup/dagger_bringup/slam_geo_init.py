#!/usr/bin/env python3
"""Start SLAM localization from GNSS + a scan-to-map search, and watch it stay locked.

The saved map is tied to the Earth by maps/<name>_geo.yaml (tools/slam_georef.py):
    p_enu = R(yaw) * p_map + t        (ENU about the datum lat/lon)
GNSS here is a 3D fix without RTK: the dual-antenna HEADING is good (~0.5 deg)
but the POSITION can be ~5 m off between sessions (lane2 vs lane3fix,
2026-09-26) -- far more than slam_toolbox's localization recovers from (it
locked 20 deg / 4.5 m wrong). So GNSS only narrows the search:

SEED (once, at startup):
  - wait for `samples` fixes taken while the rover is still, 3D fix,
    h_acc <= max_h_acc_m, fresh GNSS heading; average them;
  - search the saved map around that position (+-search_radius_m) and heading
    (+-search_yaw_deg) for the pose where the current /scan fits the walls best;
  - accept only a clear winner: score <= accept_score AND the best pose more than
    1.5 m away scores >= ambiguity_ratio x worse (a long corridor can alias);
  - publish it on /initialpose. Rejected -> retry on a newer scan; after a few
    rejections, say so: set the pose with Foxglove's pose-estimate tool (which
    publishes /initialpose too -- seen here, auto-seeding then stops).
HEALTH (continuously): the score of the scan SLAM last processed, at the pose
SLAM gave it (/slam/pose carries that scan's stamp), on /slam/match_score; /slam/localized (latched Bool) is true
after `ok_count` checks <= ok_score, false again after `lost_count` checks >=
lost_score. Scores: ~0.05-0.10 m locked, ~0.7-1.0 m wrong place.
Also /slam/gnss_pose: GNSS converted into the map, for comparison in Foxglove.

offset_x / offset_y shift the GNSS centre of the search on purpose (offline
tests). Leave them 0 on the rover.
"""
import collections
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import GPSRAW
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from std_msgs.msg import Bool, Float32

from dagger_bringup.scan_map_match import MapMatcher, scan_points

A_WGS84 = 6378137.0
E2_WGS84 = 6.69437999014e-3


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def enu_scale(lat0_deg):
    """Metres per degree (east, north) at lat0 -- same as tools/slam_georef.py."""
    lat = math.radians(lat0_deg)
    s = 1 - E2_WGS84 * math.sin(lat) ** 2
    n = A_WGS84 / math.sqrt(s)
    m = A_WGS84 * (1 - E2_WGS84) / s ** 1.5
    return math.radians(1) * n * math.cos(lat), math.radians(1) * m


class SlamGeoInit(Node):

    def __init__(self):
        super().__init__("slam_geo_init")
        self.declare_parameter("geo_file", "")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("samples", 10)
        self.declare_parameter("max_h_acc_m", 6.0)
        self.declare_parameter("max_speed", 0.05)
        self.declare_parameter("heading_timeout", 1.0)
        self.declare_parameter("search_radius_m", 8.0)
        self.declare_parameter("search_yaw_deg", 10.0)
        self.declare_parameter("accept_score", 0.15)
        self.declare_parameter("ambiguity_ratio", 2.0)
        self.declare_parameter("retry_s", 5.0)
        self.declare_parameter("health_period", 2.0)
        self.declare_parameter("ok_score", 0.20)
        self.declare_parameter("ok_count", 3)
        self.declare_parameter("lost_score", 0.35)
        self.declare_parameter("lost_count", 3)
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        with open(g("geo_file")) as f:
            geo = yaml.safe_load(f)
        self.lat0, self.lon0 = geo["datum_lat"], geo["datum_lon"]
        self.rot = math.radians(geo["yaw_deg"])
        self.tx, self.ty = geo["tx"], geo["ty"]
        self.kx, self.ky = enu_scale(self.lat0)
        self.matcher = MapMatcher(g("map_yaml"))
        self.frame = g("map_frame")
        self.samples = int(g("samples"))
        self.max_h_acc, self.max_speed = g("max_h_acc_m"), g("max_speed")
        self.heading_timeout = g("heading_timeout")
        self.radius, self.yaw_win = g("search_radius_m"), math.radians(g("search_yaw_deg"))
        self.accept, self.ambig = g("accept_score"), g("ambiguity_ratio")
        self.retry_s = g("retry_s")
        self.ok_score, self.ok_count = g("ok_score"), int(g("ok_count"))
        self.lost_score, self.lost_count = g("lost_score"), int(g("lost_count"))
        self.offset = (g("offset_x"), g("offset_y"))

        self.h_acc = None
        self.fix_type = 0
        self.speed = None
        self.heading = None        # (time, ENU yaw)
        self.scan = None
        self.scans = collections.deque(maxlen=40)   # ~4 s, to find the one /slam/pose used
        self.slam_pose = None
        self.buf = []              # still, good GNSS samples as map (x, y, yaw)
        self.seeded = False
        self.next_try = 0.0
        self.rejections = 0
        self.localized = None
        self.good = self.bad = 0

        # No TF listener: in Python it decodes every /tf message (~25 % CPU on the
        # Orin, measured live 2026-09-26) for a check that runs every 2 s.
        sensor = qos_profile_sensor_data   # mavros is best effort
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self.on_fix, sensor)
        self.create_subscription(Imu, "/gps/heading_imu", self.on_heading, sensor)
        self.create_subscription(GPSRAW, "/mavros/gpsstatus/gps1/raw", self.on_raw, sensor)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, sensor)
        self.create_subscription(LaserScan, "/scan", self.on_scan, sensor)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.on_initialpose, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/slam/pose", self.on_slam_pose, 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.gnss_pub = self.create_publisher(PoseStamped, "/slam/gnss_pose", 10)
        self.score_pub = self.create_publisher(Float32, "/slam/match_score", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.loc_pub = self.create_publisher(Bool, "/slam/localized", latched)
        self.set_localized(False)
        self.create_timer(1.0, self.try_seed)
        self.create_timer(g("health_period"), self.check_health)
        self.get_logger().info(
            f"map datum {self.lat0:.7f},{self.lon0:.7f} yaw {geo['yaw_deg']:+.2f} deg; seeding from "
            f"{self.samples} still GNSS fixes + scan search +-{self.radius:.0f} m / "
            f"+-{math.degrees(self.yaw_win):.0f} deg")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def to_map(self, lat, lon, yaw_enu):
        e = (lon - self.lon0) * self.kx - self.tx
        n = (lat - self.lat0) * self.ky - self.ty
        c, s = math.cos(self.rot), math.sin(self.rot)
        return c * e + s * n, -s * e + c * n, math.remainder(yaw_enu - self.rot, 2 * math.pi)

    def set_localized(self, value):
        if value != self.localized:
            self.localized = value
            self.loc_pub.publish(Bool(data=value))

    # ---- inputs ----
    def on_raw(self, m):
        self.fix_type, self.h_acc = m.fix_type, m.h_acc / 1000.0

    def on_odom(self, m):
        self.speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)

    def on_heading(self, m):
        self.heading = (self.now_s(), yaw_of(m.orientation))

    def on_scan(self, m):
        self.scan = m
        self.scans.append(m)

    def on_slam_pose(self, m):
        self.slam_pose = m

    def on_initialpose(self, m):
        if not self.seeded:   # someone else (Foxglove) set the pose: stop auto-seeding
            self.seeded = True
            self.get_logger().info("initial pose set externally; auto-seeding stopped")

    def on_fix(self, m):
        if self.heading is None or self.now_s() - self.heading[0] > self.heading_timeout:
            return
        x, y, yaw = self.to_map(m.latitude, m.longitude, self.heading[1])
        p = PoseStamped()
        p.header.stamp = m.header.stamp
        p.header.frame_id = self.frame
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.gnss_pub.publish(p)
        good = (m.status.status >= 0 and self.fix_type >= 3 and self.h_acc is not None
                and self.h_acc <= self.max_h_acc)
        still = self.speed is not None and self.speed < self.max_speed
        if not self.seeded:
            if good and still:
                self.buf = (self.buf + [(x, y, yaw)])[-self.samples:]
            else:
                self.buf.clear()   # only consecutive still, good fixes count

    # ---- seeding ----
    def try_seed(self):
        if self.seeded or len(self.buf) < self.samples or self.now_s() < self.next_try:
            return
        if self.scan is None or not (self.speed is not None and self.speed < self.max_speed):
            return
        if self.init_pub.get_subscription_count() < 2:   # slam_toolbox + ourselves
            self.get_logger().info("seed ready, waiting for slam_toolbox on /initialpose",
                                   throttle_duration_sec=10.0)
            return
        gx = sum(s[0] for s in self.buf) / len(self.buf) + self.offset[0]
        gy = sum(s[1] for s in self.buf) / len(self.buf) + self.offset[1]
        gyaw = math.atan2(sum(math.sin(s[2]) for s in self.buf),
                          sum(math.cos(s[2]) for s in self.buf))
        pts = scan_points(self.scan)
        if len(pts) < 50:
            self.get_logger().warn(f"scan has only {len(pts)} points, not searching")
            self.next_try = self.now_s() + self.retry_s
            return
        t0 = self.now_s()
        score, x, y, yaw, second = self.matcher.search(pts, gx, gy, gyaw, self.radius, self.yaw_win)
        dt = self.now_s() - t0
        desc = (f"best ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg) score {score:.3f} m, "
                f"next-best elsewhere {second:.3f} m; GNSS said ({gx:.2f}, {gy:.2f}, "
                f"{math.degrees(gyaw):.1f} deg) -> moved {math.hypot(x - gx, y - gy):.1f} m "
                f"[{dt:.1f} s]")
        if score > self.accept or second < self.ambig * score:
            self.rejections += 1
            self.next_try = self.now_s() + self.retry_s
            self.get_logger().warn(f"seed REJECTED ({self.rejections}): {desc}")
            if self.rejections >= 3:
                self.get_logger().warn(
                    "cannot find a clear match -- set the pose with Foxglove's pose-estimate "
                    "tool (/initialpose), or move to a spot with more walls in view",
                    throttle_duration_sec=30.0)
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.1 ** 2
        msg.pose.covariance[35] = math.radians(2) ** 2
        self.seeded = True      # before publishing: on_initialpose sees our own message
        self.init_pub.publish(msg)
        self.get_logger().info(f"seeded SLAM: {desc}")

    # ---- health ----
    def check_health(self):
        if not self.seeded or self.slam_pose is None or not self.scans:
            return
        stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9  # noqa: E731
        ts = stamp(self.slam_pose.header)
        scan = min(self.scans, key=lambda sc: abs(stamp(sc.header) - ts))
        if abs(stamp(scan.header) - ts) > 0.06:
            return   # that scan already left the buffer (or /slam/pose is stale)
        p = self.slam_pose.pose.pose
        score = self.matcher.score(scan_points(scan), p.position.x, p.position.y,
                                   yaw_of(p.orientation))
        self.score_pub.publish(Float32(data=score))
        self.good = self.good + 1 if score <= self.ok_score else 0
        self.bad = self.bad + 1 if score >= self.lost_score else 0
        if not self.localized and self.good >= self.ok_count:
            self.set_localized(True)
            self.get_logger().info(f"localized: scan fits the map (score {score:.3f} m)")
        elif self.localized and self.bad >= self.lost_count:
            self.set_localized(False)
            self.get_logger().error(f"LOST: scan no longer fits the map (score {score:.3f} m)")


def main():
    rclpy.init()
    node = SlamGeoInit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
