#!/usr/bin/env python3
"""Hold a set distance behind the followed person.

    /follow/target -> /cmd_vel_follow_raw -> collision_monitor_follow
                   -> /cmd_vel_follow -> twist_mux (30) -> cmd_vel_safety -> mavros

IT STARTS DISABLED AND STAYS THAT WAY until someone publishes true on
/follow/enable. Launching the follow stack must never be the thing that makes a
350 kg rover move; enabling it is a separate, deliberate act by the operator.
While disabled the node publishes NOTHING at all, so twist_mux times out and the
follow slot is simply absent rather than holding a zero.

IT NEVER COMMANDS REVERSE. If the target closes inside the set distance the
rover slows to a stop and waits for them to move away. Two reasons: there is no
rear sensing on this machine, and the collision monitor downstream zeroes the
whole command when its stop zone trips, reverse included -- so a reverse command
would be silently dropped exactly when it was meant to matter.

EVERYTHING HERE IS A REQUEST, NOT A GUARANTEE. collision_monitor_follow still
slows and stops this command independently, and cmd_vel_safety still zeroes it
if /scan_safety goes stale. This node is the only part of the chain that can be
wrong about where the person is; the layers below it do not trust it.
"""
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, String


class FollowController(Node):

    def __init__(self):
        super().__init__("follow_controller")

        self.declare_parameter("target_topic", "/follow/target")
        self.declare_parameter("cmd_topic", "/cmd_vel_follow_raw")
        self.declare_parameter("enable_topic", "/follow/enable")
        self.declare_parameter("estop_topic", "/estop")
        self.declare_parameter("follow_distance", 3.0)
        # Inside this band of the set distance the rover simply holds station,
        # so a walking person's natural pace variation does not make it surge.
        self.declare_parameter("distance_deadband", 0.25)
        # Expressed as the time to close the remaining gap rather than a bare
        # gain, because that is how the behaviour is actually reasoned about:
        # v = (error - deadband) / close_time. 1.5 s is roughly twice the old
        # kp_linear 0.35, so it reaches the speed cap sooner instead of easing
        # toward it while the person walks away.
        self.declare_parameter("close_time", 1.5)
        # 0.6 m/s, validated 2026-09-26: stop 0.286 m from ~0.6 m/s. The FCU's
        # WP_SPEED caps GUIDED velocity, so it must be >= this (it is 2.0).
        self.declare_parameter("max_linear", 0.6)
        # 0.5 m/s^2: 0 -> 0.4 in 0.8 s rather than 1.33 s. Measured 2026-09-25,
        # the person receded at a median 0.30 and p90 0.77 m/s, so time spent
        # ramping is time spent falling behind.
        self.declare_parameter("accel_limit", 0.5)          # m/s^2 on the request
        self.declare_parameter("kp_angular", 0.9)
        self.declare_parameter("max_angular", 0.5)
        self.declare_parameter("bearing_deadband_deg", 5.0)
        # Hysteresis on that deadband: start turning above 5 deg, stop only below
        # 3 deg. With the median bearing sitting at 4.3 deg while following
        # (2026-09-25), a single threshold plus the 0.15 rad/s stiction floor
        # made the yaw command toggle 0 <-> 0.15 whenever the reading crossed 5.
        self.declare_parameter("bearing_release_deg", 3.0)
        # Below ~0.15 rad/s the tracks do not break stiction and the commanded
        # rate mostly does not happen (measured: actual/commanded 0.12 at
        # 0.02 rad/s). Asking for less than this just winds up the FCU
        # integrator until it lets go, so ask for this much or nothing.
        self.declare_parameter("min_effective_angular", 0.15)
        # Slower than the deadband is wide: turning hard while closing in is how
        # a tracked rover ends up pointing somewhere unintended.
        self.declare_parameter("turn_speed_scale", 0.6)
        # Beyond this bearing, turn in place and do not drive at all. With the
        # LiDAR holding a target beside or behind the rover, the old rule (60 %
        # speed while turning) would drive forward PAST the person while it
        # swung round. 35 deg is a little outside the camera's 27.8 deg half
        # field of view: once the person is back in frame, driving resumes.
        self.declare_parameter("max_drive_bearing_deg", 35.0)
        self.declare_parameter("target_timeout", 0.6)
        # On losing the target, turn toward where it was last seen for this long
        # before giving up. ROTATION ONLY, never forward: turning in place with
        # nothing in view is defensible, translating blindly is not.
        # 6 s, not 4: with search_angular at 0.45 the rover sweeps ~17 deg/s, so
        # 4 s covered 70 deg and still expired in the same SECOND the selector
        # reacquired the target (2026-09-25 16:41:48) -- it disabled itself one
        # tick short of succeeding. The tradeoff is real though: this is how long
        # the rover may rotate on the spot with nothing in view, so it is a
        # parameter rather than a larger constant.
        self.declare_parameter("search_timeout", 6.0)
        # 0.25 rad/s does NOT turn this rover from a standstill. Measured on
        # 2026-09-25: two searches commanded ~53 and ~45 deg of rotation and
        # actually delivered +23.4 and -7.6 deg (44 % and 17 %). The second
        # started from exactly zero linear velocity, which is the worst case for
        # breaking track stiction on a 350 kg hull; the first still had 0.04 m/s
        # of residual motion helping it. Raised to sweep the field of view in
        # the time available -- see the steering-gain table in N6.
        self.declare_parameter("search_angular", 0.45)
        # Only worth searching if the target was actually heading out of frame.
        # Vanishing dead ahead means occlusion or out of range, and turning would
        # point the camera AWAY from where they still are.
        self.declare_parameter("search_min_bearing_deg", 8.0)
        # Click-to-follow (person_tracker) only; 0 = the behaviour above, unchanged.
        # > 0: after losing the target, stay enabled this long and LOOK for them:
        # turn toward /follow/search_hint (where the tracker thinks they are),
        # face it for search_face_hold_s, then sweep in place toward where they
        # were last seen -- ROTATION ONLY. If the tracker re-identifies them (same
        # track id) following resumes; if not, disable as before. 30 s = the
        # operator's choice (2026-09-29), matching the tracker's reid_window.
        self.declare_parameter("resume_window", 0.0)
        self.declare_parameter("search_face_deg", 10.0)
        self.declare_parameter("search_face_hold_s", 3.0)
        self.declare_parameter("hint_timeout", 0.5)
        # SLOW ZONE for FORWARD speed, applied here instead of in
        # collision_monitor_follow: Nav2 Humble's slowdown scales the WHOLE twist,
        # so any plant or wall 0.85-2.6 m ahead cut the in-place search turn to
        # 0.16 rad/s (reid1 test 2026-09-30: 0.41-0.46 with the zone clear).
        # Rotation in place is covered by the monitor's approach check (footprint
        # simulated 1.5 s ahead); the monitor keeps its STOP zone and approach.
        # Same zone, same ratio as the old PolygonSlow; a stale scan counts as
        # occupied.
        self.declare_parameter("slow_scan_topic", "/scan_safety")
        self.declare_parameter("slow_zone", [0.85, 2.60, 0.85])   # x_min, x_max, half width (m)
        self.declare_parameter("slow_ratio", 0.4)
        self.declare_parameter("slow_min_points", 3)
        self.declare_parameter("slow_scan_timeout", 0.5)
        # NO TURNING IN PLACE for a person closer than this (m from base_footprint,
        # the rotation centre) -- following, facing or searching. The footprint's
        # corners (0.85, 0.58) sweep a 1.03 m circle when the rover turns in place,
        # and /scan_safety is blind inside 1.0 m (range_min, mast self-returns), so
        # the collision monitor cannot stop a turn into someone standing there.
        # Standing still is always safe; the rover turns once they are this far
        # away. 1.6 = 1.03 + a stride. (live reid2 2026-09-30: the operator was
        # lost 1.5 m away at +45 deg and the search turned toward them.)
        self.declare_parameter("min_turn_range", 1.6)
        # ... and once holding, turn again only beyond min_turn_range + this: live
        # reid3 (2026-09-30) the operator stood ON the 1.6 m line and the rover
        # toggled hold / turn (0.16-0.32 rad/s bursts at 1.57-1.60 m).
        self.declare_parameter("min_turn_release", 0.2)
        self.declare_parameter("publish_rate", 20.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.d0 = float(g("follow_distance"))
        self.dead = float(g("distance_deadband"))
        self.close_time = max(0.1, float(g("close_time")))
        self.v_max = float(g("max_linear"))
        self.accel = float(g("accel_limit"))
        self.kp_ang = float(g("kp_angular"))
        self.w_max = float(g("max_angular"))
        self.b_dead = math.radians(float(g("bearing_deadband_deg")))
        self.b_release = math.radians(float(g("bearing_release_deg")))
        self.turning = False
        self.w_min = float(g("min_effective_angular"))
        self.turn_scale = float(g("turn_speed_scale"))
        self.max_drive_bearing = math.radians(float(g("max_drive_bearing_deg")))
        self.timeout = float(g("target_timeout"))
        self.min_turn_range = float(g("min_turn_range"))
        self.min_turn_release = float(g("min_turn_release"))
        self.last_range = math.inf
        self.too_close = False             # inside a too-close episode (hysteresis + one log)
        self.search_timeout = float(g("search_timeout"))
        self.search_w = float(g("search_angular"))
        self.search_min_bearing = math.radians(float(g("search_min_bearing_deg")))
        self.resume_window = float(g("resume_window"))
        self.face_tol = math.radians(float(g("search_face_deg")))
        self.face_hold = float(g("search_face_hold_s"))
        self.hint_timeout = float(g("hint_timeout"))
        self.slow_zone = [float(v) for v in g("slow_zone")]
        self.slow_ratio = float(g("slow_ratio"))
        self.slow_min_points = int(g("slow_min_points"))
        self.slow_scan_timeout = float(g("slow_scan_timeout"))
        self.slow_pts, self.slow_time = 0, 0.0
        self.hint = None                   # (bearing, monotonic receipt, range) from the tracker
        self.face_since = None
        self.sweeping = False              # once sweeping, the hint no longer pulls the rover back
        self.sweep_dir = 1.0
        self.rate = float(g("publish_rate"))

        self.enabled = False
        self.estop = False
        self.target = None
        self.target_time = 0.0
        self.v_cmd = 0.0
        # While not following we publish an explicit zero until this time, then
        # go quiet so twist_mux releases the follow slot rather than us pinning
        # it at priority 30. Zero at startup = silent until something happens.
        self.stop_until = 0.0
        self.search_until = 0.0
        self.last_bearing = 0.0
        # Auto-disable must mean "I lost the target I had", never "I have not
        # been given one yet". Without this the controller switched itself off on
        # the first tick after being enabled, before any target had arrived.
        self.had_target = False
        # The track id this follow session belongs to, latched from the first id
        # seen after enabling. If the selector's id changes -- a different person
        # -- follow disables itself. Without this a bystander could be handed over
        # seamlessly: when a lock expires with someone else in frame, the
        # selector publishes the new person with no gap, so the controller never
        # saw a loss at all.
        self.locked_id = None
        self.last_state = None
        # For the operator's UI: current state and why follow last turned off.
        # Publishing only -- nothing below reads it back.
        self.disable_reason = "not enabled yet"
        self.state_sent, self.state_sent_at = None, 0.0

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Twist, g("cmd_topic"), 10)
        self.state_pub = self.create_publisher(String, "/follow/state", 10)
        self.create_subscription(PoseStamped, g("target_topic"), self.on_target, be)
        self.create_subscription(Bool, g("enable_topic"), self.on_enable, 10)
        self.create_subscription(Bool, g("estop_topic"), self.on_estop, 10)
        self.create_subscription(Int32, "/follow/track_id", self.on_track_id, 10)
        self.create_subscription(PoseStamped, "/follow/search_hint", self.on_hint, be)
        self.create_subscription(LaserScan, g("slow_scan_topic"), self.on_scan, be)
        self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().warn(
            f"follow_controller is DISABLED. It will publish nothing until "
            f"'{g('enable_topic')}' goes true. Set distance {self.d0} m "
            f"(+-{self.dead}), max {self.v_max} m/s / {self.w_max} rad/s, no reverse.")

    def on_enable(self, msg):
        if msg.data == self.enabled:
            return
        self.enabled = msg.data
        if self.enabled:
            self.had_target = False
            self.locked_id = None
            self.search_until = 0.0
            self.get_logger().warn("FOLLOW ENABLED -- the rover will now drive itself "
                                   "toward the selected target")
        else:
            self.v_cmd = 0.0
            self.stop_until = time.monotonic() + 1.0
            self.disable_reason = "operator"
            self.get_logger().warn("follow disabled -- commanding zero, then releasing "
                                   "the twist_mux slot")

    def on_track_id(self, msg):
        if not self.enabled:
            return
        if self.locked_id is None:
            self.locked_id = msg.data
            return
        if msg.data != self.locked_id:
            self.enabled = False
            self.had_target = False
            self.search_until = 0.0
            self.v_cmd = 0.0
            self.stop_until = time.monotonic() + 1.0
            self.disable_reason = f"target changed (#{self.locked_id} -> #{msg.data})"
            self.get_logger().error(
                f"TARGET CHANGED to a different person / marker (track {self.locked_id} -> "
                f"{msg.data}) -- FOLLOW DISABLED. Re-enable explicitly to follow them.")

    def on_estop(self, msg):
        if msg.data and not self.estop:
            self.get_logger().error("E-STOP -- follow controller standing down")
            self.v_cmd = 0.0
        self.estop = msg.data

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + np.arange(len(r), dtype=np.float32) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        x, y = r * np.cos(a), r * np.sin(a)
        x0, x1, hw = self.slow_zone
        self.slow_pts = int((ok & (x >= x0) & (x <= x1) & (np.abs(y) <= hw)).sum())
        self.slow_time = time.monotonic()

    def slowed(self, now):
        return (self.slow_pts >= self.slow_min_points
                or now - self.slow_time > self.slow_scan_timeout)

    def on_hint(self, msg):
        p = msg.pose.position
        self.hint = (math.atan2(p.y, p.x), time.monotonic(), math.hypot(p.x, p.y))

    def on_target(self, msg):
        self.target = msg
        self.target_time = time.monotonic()
        self.had_target = True
        p = msg.pose.position
        self.last_bearing = math.atan2(p.y, p.x)
        self.last_range = math.hypot(p.x, p.y)

    def state(self):
        now = time.monotonic()
        if self.estop:
            return "estop"
        if not self.enabled:
            return "disabled"
        if self.target is not None and now - self.target_time <= self.timeout:
            return "following"
        if self.search_until and now < self.search_until:
            return "searching"
        return "no target"

    def publish_state(self, st, now):
        """/follow/state for the UI: on every change and at 2 Hz."""
        text = st if st != "disabled" else f"disabled: {self.disable_reason}"
        if text != self.state_sent or now - self.state_sent_at >= 0.5:
            self.state_pub.publish(String(data=text))
            self.state_sent, self.state_sent_at = text, now

    def too_close_to_turn(self, rng, now):
        """True (and says so, once) from inside min_turn_range until the person is
        beyond min_turn_range + min_turn_release."""
        if rng >= self.min_turn_range + (self.min_turn_release if self.too_close else 0.0):
            self.too_close = False
            return False
        if not self.too_close:
            self.too_close = True
            self.get_logger().warn(
                f"person {rng:.1f} m away -- too close to turn in place (corners sweep "
                f"1.03 m, the safety scan is blind inside 1.0 m): holding still until "
                f"they are {self.min_turn_range + self.min_turn_release:.1f} m away")
        return True

    def search_rate(self, now):
        """Click-to-follow search: face the tracker's hint, hold, then sweep in ONE
        direction (the side they were lost on) until found or the window ends."""
        fresh = self.hint is not None and now - self.hint[1] <= self.hint_timeout
        # Last SEEN (last target) within min_turn_range, or expected there now: they
        # are probably still right beside the rover, invisible to the camera (out
        # of view) and the LiDAR (< 1.0 m). Never turn on the hint alone -- it is
        # an extrapolation (hug scene: it drifted to 1.9 m while A stood at 0.9 m).
        if self.too_close_to_turn(min(self.last_range, self.hint[2] if fresh else math.inf), now):
            return 0.0                      # wait for re-identification without turning
        if not self.sweeping and fresh:
            b = self.hint[0]
            if abs(b) > self.face_tol:
                self.face_since = None
                self.sweep_dir = math.copysign(1.0, b)
                return math.copysign(self.search_w, b)
            if self.face_since is None:
                self.face_since = now
            if now - self.face_since < self.face_hold:
                return 0.0                  # facing where they should be: let the camera look
        if not self.sweeping:
            self.sweeping = True
            self.get_logger().warn("not where the tracker expected -- sweeping in place "
                                   f"({'left' if self.sweep_dir > 0 else 'right'})")
        return self.sweep_dir * self.search_w

    def tick(self):
        now = time.monotonic()
        have_target = (self.target is not None
                       and now - self.target_time <= self.timeout)

        # Decide, at the moment the target is lost, whether a search is worth it.
        if self.enabled and not self.estop and not have_target \
                and self.last_state == "following":
            if self.resume_window > 0:
                self.search_until = now + self.resume_window
                self.face_since = None
                self.sweeping = False
                self.sweep_dir = math.copysign(1.0, self.last_bearing)   # the side they left by
                self.get_logger().warn(
                    f"target lost at {math.degrees(self.last_bearing):+.1f} deg -- searching "
                    f"(turn to where the tracker expects them, then sweep) for "
                    f"{self.resume_window:.0f} s; following resumes if they are re-identified. "
                    f"No forward motion.")
            elif abs(self.last_bearing) >= self.search_min_bearing:
                self.search_until = now + self.search_timeout
                self.get_logger().warn(
                    f"target lost at {math.degrees(self.last_bearing):+.1f} deg -- "
                    f"turning toward it for {self.search_timeout:.0f} s, no forward motion")
            else:
                self.search_until = 0.0
                self.get_logger().warn(
                    "target lost near dead ahead -- occlusion or out of range, not out "
                    "of frame, so not turning")

        st = self.state()
        self.publish_state(st, now)
        if st != self.last_state:
            if self.last_state == "following":
                self.stop_until = now + 1.0
            elif self.last_state == "searching" and st == "following":
                self.get_logger().info("target reacquired")
            self.last_state = st

        if st == "searching":
            # Rotate toward the last known bearing. Zero linear, always.
            self.v_cmd = 0.0
            cmd = Twist()
            if self.resume_window > 0:
                cmd.angular.z = self.search_rate(now)
            elif not self.too_close_to_turn(self.last_range, now):
                cmd.angular.z = math.copysign(self.search_w, self.last_bearing)
            self.pub.publish(cmd)
            return

        if st == "no target" and self.enabled and self.had_target:
            # Searching either failed or was not attempted: this is a real loss.
            # Disable rather than sit armed and waiting -- the operator chose
            # explicit re-enable so the rover never picks a target on its own.
            self.enabled = False
            self.had_target = False
            self.search_until = 0.0
            self.stop_until = now + 1.0
            self.disable_reason = "target lost"
            self.get_logger().error(
                "target not recovered -- FOLLOW DISABLED. Re-enable explicitly with "
                "/follow/enable true once you are back in front of the rover.")

        if st != "following":
            self.v_cmd = 0.0
            if now < self.stop_until:
                self.pub.publish(Twist())
            return
        self.search_until = 0.0

        p = self.target.pose.position
        rng = math.hypot(p.x, p.y)
        bearing = math.atan2(p.y, p.x)

        err = rng - self.d0
        v_want = (0.0 if abs(err) < self.dead
                  else (err - math.copysign(self.dead, err)) / self.close_time)
        v_want = max(0.0, min(self.v_max, v_want))      # never reverse

        close = self.too_close_to_turn(rng, now)
        if close:
            self.turning = False
            v_want = 0.0                            # hold still: no turn, no creeping
        elif self.turning:
            self.turning = abs(bearing) >= self.b_release
        else:
            self.turning = abs(bearing) >= self.b_dead
        if not self.turning:
            w = 0.0
        else:
            w = self.kp_ang * bearing
            w = math.copysign(max(self.w_min, min(self.w_max, abs(w))), w)
            v_want *= self.turn_scale
            if abs(bearing) > self.max_drive_bearing:
                v_want = 0.0                        # face them first, then drive

        # Ramp the request so a target reacquired at distance does not step the
        # command; the FCU speed loop handles a ramp far better than a jump.
        dv = self.accel / self.rate
        self.v_cmd += max(-dv, min(dv, v_want - self.v_cmd))
        if close:
            self.v_cmd = 0.0                        # a stop, like every other stop here
        if self.slowed(now):
            # instant, like the collision monitor's slowdown was; the ramp above
            # then brings the speed back up smoothly once the zone clears
            self.v_cmd = min(self.v_cmd, max(v_want, 0.0) * self.slow_ratio)

        cmd = Twist()
        cmd.linear.x = self.v_cmd
        cmd.angular.z = w
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
