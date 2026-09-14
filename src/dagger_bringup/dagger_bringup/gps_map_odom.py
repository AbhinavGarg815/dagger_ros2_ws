#!/usr/bin/env python3
"""GPS global layer: publish map->odom from ArduPilot EK3 vs our fused odom.

Frames (REP-105):
  map  = ArduPilot EK3 LOCAL frame (ENU, origin = EKF origin). This is the frame
         /mavros/local_position/odom is expressed in and the frame Mission Planner
         AUTO effectively navigates in -> Nav2 GPS goals are directly comparable.
  odom = our fused odometry (EK3 velocity + raw gyro, never jumps).

  map->odom = T(map, base)[EK3 pose]  *  inverse(T(odom, base)[fused odom])

The raw correction moves with GPS noise (~0.1 m jitter, ~1 m/min walk without
RTK), so it is LOW-PASS filtered (time constant tau_xy / tau_yaw) before being
broadcast: the controller keeps tracking the smooth odom frame, and GPS drags
the global frame into place slowly instead of jerking the path.

Gating (a sample is only used when):
  - it is fresh; odom is INTERPOLATED at the EK3 sample's stamp (no skew error),
  - |yaw rate| < max_yaw_rate (during pivots EK3/odom stamp skew and the EK3
    velocity lever-arm transient make the pair inconsistent),
  - GNSS fix >= 3D and h_acc <= max_h_acc,
  - for the YAW part only: dual-antenna GPS heading valid (GPS_RAW yaw not
    0/65535); without it EK3 yaw drifts, so the yaw offset is HELD.
map->odom is not published until the first sample with a valid GPS heading.
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import GPSRAW
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose_offset(mx, my, myaw, ox, oy, oyaw):
    """map->odom (x, y, yaw) given base in map (m*) and base in odom (o*)."""
    yaw = wrap(myaw - oyaw)
    c, s = math.cos(yaw), math.sin(yaw)
    return mx - (c * ox - s * oy), my - (s * ox + c * oy), yaw


class GpsMapOdom(Node):

    def __init__(self):
        super().__init__("gps_map_odom")
        self.declare_parameter("ek3_topic", "/mavros/local_position/odom")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("gps_raw_topic", "/mavros/gpsstatus/gps1/raw")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("tau_xy", 5.0)        # s, translation low-pass
        self.declare_parameter("tau_yaw", 10.0)      # s, yaw low-pass
        self.declare_parameter("max_yaw_rate", 0.15)  # rad/s, gate during turns
        self.declare_parameter("max_h_acc", 5.0)     # m
        self.declare_parameter("tf_future", 0.1)     # s, post-date like AMCL
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame, self.odom_frame = g("map_frame"), g("odom_frame")
        self.tau_xy, self.tau_yaw = float(g("tau_xy")), float(g("tau_yaw"))
        self.max_wz = float(g("max_yaw_rate"))
        self.max_h_acc, self.tf_future = float(g("max_h_acc")), float(g("tf_future"))

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=20)
        self.ek3 = []    # recent (t, x, y, yaw)
        self.odom = None  # latest (t, x, y, yaw, wz)
        self.odom_hist = []
        self.last_ek3_t = None
        self.gps_ok = False
        self.gps_yaw_ok = False
        self.create_subscription(Odometry, g("ek3_topic"), self.on_ek3, be)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, be)
        self.create_subscription(GPSRAW, g("gps_raw_topic"), self.on_gps, be)
        self.br = TransformBroadcaster(self)

        self.off = None       # filtered map->odom (x, y, yaw)
        self.last_upd = None  # time of last filter update
        self.stats = {"used": 0, "yaw_held": 0, "gated": 0}
        self.create_timer(1.0 / float(g("rate")), self.tick)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"gps_map_odom: map=EK3 local frame, tau_xy={self.tau_xy}s tau_yaw={self.tau_yaw}s; "
            "waiting for a GPS-heading-valid sample before publishing map->odom")

    def on_ek3(self, m):
        p = m.pose.pose
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        self.ek3.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        self.ek3 = self.ek3[-40:]

    def on_odom(self, m):
        p = m.pose.pose
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        self.odom = (t, p.position.x, p.position.y, yaw_of(p.orientation), m.twist.twist.angular.z)
        self.odom_hist.append(self.odom)
        self.odom_hist = self.odom_hist[-60:]

    def odom_at(self, t):
        """Odom pose interpolated at time t (None if t is outside the history)."""
        h = self.odom_hist
        if len(h) < 2 or t < h[0][0]:
            return None
        if t > h[-1][0]:
            # EK3 and odom are both stamped ~"now", so the EK3 sample is often a few
            # ms NEWER than the newest odom sample. Up to 50 ms, use the newest odom
            # (error = speed * 50 ms, mm-level at our speeds); beyond that, no match.
            return h[-1][1:] if t - h[-1][0] <= 0.05 else None
        for a, b in zip(h[:-1], h[1:]):
            if a[0] <= t <= b[0]:
                f = (t - a[0]) / max(b[0] - a[0], 1e-6)
                return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]),
                        wrap(a[3] + f * wrap(b[3] - a[3])), b[4])
        return None

    def on_gps(self, m):
        self.gps_ok = m.fix_type >= 3 and m.h_acc / 1000.0 <= self.max_h_acc
        self.gps_yaw_ok = m.yaw not in (0, 65535)

    def tick(self):
        now = self.get_clock().now()
        self.update(now.nanoseconds * 1e-9)
        if self.off is None:
            return
        x, y, yaw = self.off
        t = TransformStamped()
        t.header.stamp = (now + Duration(seconds=self.tf_future)).to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.br.sendTransform(t)

    def update(self, now_s):
        if self.odom is None or not self.ek3:
            return
        # Latest EK3 sample, paired with our odom interpolated at ITS stamp.
        te, mx, my, myaw = self.ek3[-1]
        if te == self.last_ek3_t:
            return  # nothing new
        o = self.odom_at(te)
        if o is None and now_s - te < 0.3:
            return  # odom for this stamp not in yet -- retry on the next tick
        self.last_ek3_t = te
        if o is None or not self.gps_ok or now_s - te > 0.5 or abs(o[3]) > self.max_wz:
            self.stats["gated"] += 1
            return
        ox, oy, oyaw, _ = o
        rx, ry, ryaw = compose_offset(mx, my, myaw, ox, oy, oyaw)
        if self.off is None:
            if not self.gps_yaw_ok:
                self.stats["gated"] += 1
                return
            self.off, self.last_upd = (rx, ry, ryaw), now_s
            self.get_logger().info(
                f"map->odom INITIALISED: ({rx:.2f}, {ry:.2f}) yaw {math.degrees(ryaw):.1f} deg")
            return
        dt = max(0.0, min(now_s - self.last_upd, 1.0))
        self.last_upd = now_s
        x, y, yaw = self.off
        if self.gps_yaw_ok:
            yaw = wrap(yaw + (1 - math.exp(-dt / self.tau_yaw)) * wrap(ryaw - yaw))
        else:
            self.stats["yaw_held"] += 1  # EK3 yaw drifts without GPS heading: hold ours
        # Translation target computed with the yaw we PUBLISH (filtered/held), so the
        # base lands exactly on the EK3 position under the broadcast rotation.
        c, s = math.cos(yaw), math.sin(yaw)
        rx, ry = mx - (c * ox - s * oy), my - (s * ox + c * oy)
        a = 1 - math.exp(-dt / self.tau_xy)
        self.off = (x + a * (rx - x), y + a * (ry - y), yaw)
        self.stats["used"] += 1
        self.raw = (rx, ry, ryaw)

    def report(self):
        if self.off is None:
            self.get_logger().warn(
                f"map->odom NOT published yet (gps_ok={self.gps_ok}, gps_yaw_ok={self.gps_yaw_ok}, "
                f"gated={self.stats['gated']})")
            return
        x, y, yaw = self.off
        rx, ry, ryaw = getattr(self, "raw", self.off)
        self.get_logger().info(
            f"map->odom ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.2f} | raw-filtered "
            f"d=({rx - x:+.2f}, {ry - y:+.2f}) dyaw {math.degrees(wrap(ryaw - yaw)):+.2f} deg | "
            f"used {self.stats['used']} gated {self.stats['gated']} yaw_held {self.stats['yaw_held']}")
        self.stats = {"used": 0, "yaw_held": 0, "gated": 0}


def main(args=None):
    rclpy.init(args=args)
    node = GpsMapOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
