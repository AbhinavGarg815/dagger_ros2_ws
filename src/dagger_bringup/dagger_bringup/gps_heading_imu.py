#!/usr/bin/env python3
"""Dual-antenna GNSS heading -> sensor_msgs/Imu (yaw only) for the fused odom EKF.

The CUAV C-RTK 2HP moving-baseline heading (GPS_RAW_INT.yaw, centi-degrees,
compass/NED) is absolute and drift-free, and fusing it removed gyro drift (~7 deg
over 3 min of pivoting on 2026-09-14). But fused RAW it rotated the costmaps.

WHY IT IS FILTERED (2026-09-26). The costmaps live in the odom frame, and odom
heading followed every wobble of the GNSS heading almost one-for-one: over 30 s
with the rover stationary the gyro integrated +0.01 deg, while the fused yaw
spanned 1.70 deg in steps of up to 0.95 deg -- matching the GNSS input's 1.71
deg span. Each wobble rotated the whole map, walls were re-marked a degree
round, and the old marks, now just behind the new ones, could never be cleared:
the operator saw "2-3 maps in rotated manner" in the global costmap. Worse on
every T2 start: the EKF starts at yaw 0, the costmaps mark the first scans at
that heading, then the first GNSS heading snaps everything ~48 deg round.

So the heading is now a COMPLEMENTARY FILTER: the gyro is integrated
continuously, and the GNSS only slowly corrects the offset between gyro heading
and true heading (time constant offset_tau, 20 s). The output is as smooth as
the gyro and as drift-free as the GNSS. Samples disagreeing with the estimate by
more than outlier_deg are ignored unless they persist for reseed_s, which
means the offset genuinely moved (e.g. after a long dropout) and it re-seeds.

Startup: the offset is seeded from seed_samples readings, not one. Once locked
-- and again on any re-seed, which is by definition a heading jump -- the node
clears both costmaps once, so marks made at the wrong heading do not survive.

Gated: nothing is published while the rover turns (|gyro z| > max_yaw_rate):
the GNSS heading lags the gyro by 0.2-0.5 s. The EKF integrates the gyro meanwhile.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavros_msgs.msg import GPSRAW
from nav2_msgs.srv import ClearEntireCostmap
from sensor_msgs.msg import Imu


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class GpsHeadingImu(Node):

    def __init__(self):
        super().__init__("gps_heading_imu")
        self.declare_parameter("gps_raw_topic", "/mavros/gpsstatus/gps1/raw")
        self.declare_parameter("gyro_topic", "/mavros/imu/data_raw")
        self.declare_parameter("output_topic", "/gps/heading_imu")
        self.declare_parameter("frame_id", "base_footprint")
        # Uncertainty of the FILTERED heading handed to the EKF.
        self.declare_parameter("yaw_stddev_deg", 0.5)
        self.declare_parameter("max_yaw_rate", 0.10)  # rad/s
        self.declare_parameter("offset_tau", 20.0)    # s
        self.declare_parameter("seed_samples", 10)    # ~2 s at 5 Hz
        self.declare_parameter("outlier_deg", 10.0)
        self.declare_parameter("reseed_s", 5.0)
        self.declare_parameter("clear_costmaps_on_lock", True)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.frame = g("frame_id")
        self.yaw_var = math.radians(float(g("yaw_stddev_deg"))) ** 2
        self.max_wz = float(g("max_yaw_rate"))
        self.tau = float(g("offset_tau"))
        self.n_seed = int(g("seed_samples"))
        self.outlier = math.radians(float(g("outlier_deg")))
        self.reseed_s = float(g("reseed_s"))
        self.clear_on_lock = bool(g("clear_costmaps_on_lock"))

        self.wz = 0.0
        self.gyro_yaw = 0.0          # integrated raw gyro, arbitrary origin
        self.t_gyro = None
        self.offset = None           # true heading - gyro heading
        self.seed = []
        self.t_last_fix = None
        self.outlier_since = None
        self.n_pub = self.n_turning = self.n_invalid = self.n_outlier = 0

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Imu, g("output_topic"), 10)
        self.create_subscription(Imu, g("gyro_topic"), self.on_gyro, be)
        self.create_subscription(GPSRAW, g("gps_raw_topic"), self.on_gps, be)
        self.clear_cli = [self.create_client(ClearEntireCostmap, s) for s in
                          ("/local_costmap/clear_entirely_local_costmap",
                           "/global_costmap/clear_entirely_global_costmap")]
        self.clear_timer = None
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"gps_heading_imu -> {g('output_topic')}: gyro + GNSS complementary filter, "
            f"tau {self.tau:.0f} s, output sd {g('yaw_stddev_deg')} deg, gated |wz| < "
            f"{self.max_wz} rad/s; waiting for {self.n_seed} headings to lock")

    def on_gyro(self, m):
        now = time.monotonic()
        if self.t_gyro is not None:
            dt = min(0.2, now - self.t_gyro)   # a stalled stream must not jump
            self.gyro_yaw = wrap(self.gyro_yaw + m.angular_velocity.z * dt)
        self.t_gyro = now
        self.wz = m.angular_velocity.z

    def on_gps(self, m):
        if m.yaw in (0, 65535):
            self.n_invalid += 1
            return
        if abs(self.wz) > self.max_wz:
            self.n_turning += 1
            return
        if self.t_gyro is None:
            return
        # GPS_RAW yaw: degrees clockwise from north -> ENU yaw: CCW from east.
        h = wrap(math.radians(90.0 - m.yaw / 100.0))
        meas = wrap(h - self.gyro_yaw)
        now = time.monotonic()

        if self.offset is None:
            self.seed.append(meas)
            if len(self.seed) < self.n_seed:
                return
            s = sum(math.sin(x) for x in self.seed); c = sum(math.cos(x) for x in self.seed)
            self.offset = math.atan2(s, c)
            spread = max(abs(wrap(x - self.offset)) for x in self.seed)
            self.seed = []
            self.t_last_fix = now
            self.get_logger().info(
                f"heading LOCKED at {math.degrees(wrap(self.gyro_yaw + self.offset)):.1f} deg "
                f"(seed spread +-{math.degrees(spread):.1f} deg)")
            self.schedule_clear("heading lock")
        else:
            err = wrap(meas - self.offset)
            if abs(err) > self.outlier:
                self.n_outlier += 1
                if self.outlier_since is None:
                    self.outlier_since = now
                elif now - self.outlier_since > self.reseed_s:
                    self.get_logger().warn(
                        f"GNSS heading has disagreed by {math.degrees(err):+.1f} deg for "
                        f"{self.reseed_s:.0f} s -- re-seeding (a heading jump)")
                    self.offset = None
                    self.outlier_since = None
                return
            self.outlier_since = None
            dt = now - (self.t_last_fix or now)
            a = min(1.0, max(dt, 0.05) / self.tau)
            self.offset = wrap(self.offset + a * err)
            self.t_last_fix = now

        yaw = wrap(self.gyro_yaw + self.offset)
        out = Imu()
        out.header.stamp = m.header.stamp
        out.header.frame_id = self.frame
        out.orientation.z, out.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        out.orientation_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, self.yaw_var]
        out.angular_velocity_covariance[0] = -1.0     # not provided
        out.linear_acceleration_covariance[0] = -1.0  # not provided
        self.pub.publish(out)
        self.n_pub += 1

    def schedule_clear(self, why):
        # Give the EKF ~2 s to converge onto the new heading before clearing.
        if not self.clear_on_lock:
            return
        if self.clear_timer is not None:
            self.clear_timer.cancel()
        self.clear_why = why
        self.clear_timer = self.create_timer(2.0, self.do_clear)

    def do_clear(self):
        self.clear_timer.cancel()
        self.clear_timer = None
        done = []
        for cli in self.clear_cli:
            if cli.service_is_ready():
                cli.call_async(ClearEntireCostmap.Request())
                done.append(cli.srv_name.split("/")[1])
        self.get_logger().info(
            f"cleared {', '.join(done) or 'no costmaps (not up yet)'} after {self.clear_why}: "
            f"marks made at the pre-lock heading would otherwise survive as a rotated copy")

    def report(self):
        state = ("locked" if self.offset is not None else
                 f"seeding {len(self.seed)}/{self.n_seed}")
        self.get_logger().info(
            f"{state}; heading published {self.n_pub}, skipped turning {self.n_turning}, "
            f"outliers {self.n_outlier}, invalid {self.n_invalid} (last 10 s)")
        self.n_pub = self.n_turning = self.n_invalid = self.n_outlier = 0


def main(args=None):
    rclpy.init(args=args)
    node = GpsHeadingImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
