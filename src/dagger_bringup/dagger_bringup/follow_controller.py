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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool


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
        self.declare_parameter("distance_deadband", 0.35)
        self.declare_parameter("kp_linear", 0.35)
        # 0.4 m/s is where the FCU cruise is calibrated (CRUISE_SPEED 0.4 /
        # CRUISE_THROTTLE 45) and where the 0.14 m stop distance was measured.
        self.declare_parameter("max_linear", 0.4)
        self.declare_parameter("accel_limit", 0.3)          # m/s^2 on the request
        self.declare_parameter("kp_angular", 0.9)
        self.declare_parameter("max_angular", 0.5)
        self.declare_parameter("bearing_deadband_deg", 5.0)
        # Below ~0.15 rad/s the tracks do not break stiction and the commanded
        # rate mostly does not happen (measured: actual/commanded 0.12 at
        # 0.02 rad/s). Asking for less than this just winds up the FCU
        # integrator until it lets go, so ask for this much or nothing.
        self.declare_parameter("min_effective_angular", 0.15)
        # Slower than the deadband is wide: turning hard while closing in is how
        # a tracked rover ends up pointing somewhere unintended.
        self.declare_parameter("turn_speed_scale", 0.6)
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
        self.declare_parameter("publish_rate", 20.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.d0 = float(g("follow_distance"))
        self.dead = float(g("distance_deadband"))
        self.kp_lin = float(g("kp_linear"))
        self.v_max = float(g("max_linear"))
        self.accel = float(g("accel_limit"))
        self.kp_ang = float(g("kp_angular"))
        self.w_max = float(g("max_angular"))
        self.b_dead = math.radians(float(g("bearing_deadband_deg")))
        self.w_min = float(g("min_effective_angular"))
        self.turn_scale = float(g("turn_speed_scale"))
        self.timeout = float(g("target_timeout"))
        self.search_timeout = float(g("search_timeout"))
        self.search_w = float(g("search_angular"))
        self.search_min_bearing = math.radians(float(g("search_min_bearing_deg")))
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
        self.last_state = None

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Twist, g("cmd_topic"), 10)
        self.create_subscription(PoseStamped, g("target_topic"), self.on_target, be)
        self.create_subscription(Bool, g("enable_topic"), self.on_enable, 10)
        self.create_subscription(Bool, g("estop_topic"), self.on_estop, 10)
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
            self.search_until = 0.0
            self.get_logger().warn("FOLLOW ENABLED -- the rover will now drive itself "
                                   "toward the tracked person")
        else:
            self.v_cmd = 0.0
            self.stop_until = time.monotonic() + 1.0
            self.get_logger().warn("follow disabled -- commanding zero, then releasing "
                                   "the twist_mux slot")

    def on_estop(self, msg):
        if msg.data and not self.estop:
            self.get_logger().error("E-STOP -- follow controller standing down")
            self.v_cmd = 0.0
        self.estop = msg.data

    def on_target(self, msg):
        self.target = msg
        self.target_time = time.monotonic()
        self.had_target = True
        p = msg.pose.position
        self.last_bearing = math.atan2(p.y, p.x)

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

    def tick(self):
        now = time.monotonic()
        have_target = (self.target is not None
                       and now - self.target_time <= self.timeout)

        # Decide, at the moment the target is lost, whether a search is worth it.
        if self.enabled and not self.estop and not have_target \
                and self.last_state == "following":
            if abs(self.last_bearing) >= self.search_min_bearing:
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
        v_want = 0.0 if abs(err) < self.dead else self.kp_lin * (err - math.copysign(self.dead, err))
        v_want = max(0.0, min(self.v_max, v_want))      # never reverse

        if abs(bearing) < self.b_dead:
            w = 0.0
        else:
            w = self.kp_ang * bearing
            w = math.copysign(max(self.w_min, min(self.w_max, abs(w))), w)
            v_want *= self.turn_scale

        # Ramp the request so a target reacquired at distance does not step the
        # command; the FCU speed loop handles a ramp far better than a jump.
        dv = self.accel / self.rate
        self.v_cmd += max(-dv, min(dv, v_want - self.v_cmd))

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
