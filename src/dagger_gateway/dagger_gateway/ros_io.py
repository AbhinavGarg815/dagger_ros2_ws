"""The gateway's ROS side: topics in, STOP actions out. Runs in the rclpy thread.

STOP (what tools/estop.sh does, plus the job-level stops), in this order, nothing
waited on:
    /estop true, then streamed at 10 Hz while latched  (twist_mux lock + cmd_vel_safety zero)
    FCU -> HOLD                                        (/mavros/set_mode)
    cancel every Nav2 goal                             (navigate_to_pose, navigate_through_poses)
    /follow/enable false x3                            (a single message can be dropped)
THE LATCH: any /estop true (ours, tools/estop.sh, Foxglove) latches STOPPED; any
/estop false (tools/estop_clear.sh -- the gateway never sends one in v1) releases
it and the gateway stops streaming at once, so the clear is not overridden. The
mode stays HOLD: the engineer re-selects GUIDED.
--stop-dry-run: STOP changes the operating state only; nothing is published or
called. /estop is still listened to.
"""
import time

from action_msgs.srv import CancelGoal
from mavros_msgs.srv import SetMode
from rclpy.node import Node
from std_msgs.msg import Bool, String

FOLLOW_OFF_GRACE_S = 0.7   # ignore "following" from /follow/state this long after we turned it off
FOLLOW_STATE_TIMEOUT_S = 2.0   # follow_controller publishes /follow/state at 2 Hz


class RosIO(Node):

    def __init__(self):
        super().__init__("dagger_gateway")
        p = self.declare_parameter
        p("host", "0.0.0.0")
        p("port", 8090)
        p("rover_id", "dagger-1")
        p("rover_name", "Dagger 1")
        p("stop_dry_run", False)
        p("heartbeat_period_ms", 250)
        p("state_period_ms", 200)
        p("follow_stop_after_silence_ms", 1500)
        p("route_pause_after_silence_ms", 3000)
        # top speed of any job: follow_controller max_linear 0.6 (Nav2 drives at 0.4)
        p("max_speed_mps", 0.6)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.param = g
        self.dry_run = bool(g("stop_dry_run"))
        self.on_estop_edge = None          # set by the gateway: called (in this thread) with True/False

        self.latched = False
        self.follow_state = None           # (text, monotonic s)
        self.follow_off_at = -1e9

        self.estop_pub = self.create_publisher(Bool, "/estop", 10)
        self.follow_pub = self.create_publisher(Bool, "/follow/enable", 10)
        self.create_subscription(Bool, "/estop", self.on_estop, 10)
        self.create_subscription(String, "/follow/state", self.on_follow_state, 10)
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.cancel_clis = [self.create_client(CancelGoal, f"/{a}/_action/cancel_goal")
                            for a in ("navigate_to_pose", "navigate_through_poses")]
        self.create_timer(0.1, self.stream_estop)

    # ---------------- in ----------------
    def on_estop(self, m):
        if m.data == self.latched:
            return
        self.latched = m.data
        self.get_logger().warn("/estop true: STOPPED latched" if m.data else
                               "/estop false: latch released (mode stays HOLD)")
        if self.on_estop_edge:
            self.on_estop_edge(m.data)

    def on_follow_state(self, m):
        self.follow_state = (m.data, time.monotonic())

    def following(self):
        """follow_controller is enabled, per its own /follow/state (2 Hz)."""
        if self.follow_state is None:
            return False
        text, t = self.follow_state
        now = time.monotonic()
        if now - t > FOLLOW_STATE_TIMEOUT_S or text.startswith("disabled"):
            return False
        return t - self.follow_off_at > FOLLOW_OFF_GRACE_S

    # ---------------- out ----------------
    def stream_estop(self):
        if self.latched and not self.dry_run:
            self.estop_pub.publish(Bool(data=True))

    def stop(self):
        if self.dry_run:
            self.get_logger().warn("STOP (dry run: nothing published, FCU untouched)")
            return
        self.get_logger().error("STOP: /estop latched, FCU -> HOLD, Nav2 goals cancelled, follow off")
        self.latched = True
        self.estop_pub.publish(Bool(data=True))
        if self.mode_cli.service_is_ready():
            fut = self.mode_cli.call_async(SetMode.Request(base_mode=0, custom_mode="HOLD"))
            fut.add_done_callback(self._mode_done)
        else:
            self.get_logger().error("STOP: /mavros/set_mode not available -- FCU NOT set to HOLD")
        for cli in self.cancel_clis:
            if cli.service_is_ready():
                cli.call_async(CancelGoal.Request())     # empty request = every goal
        self.follow_off("STOP")

    def _mode_done(self, fut):
        r = fut.result()
        if r is None or not r.mode_sent:
            self.get_logger().error("STOP: FCU did not take HOLD")

    def follow_off(self, why):
        if self.dry_run and why == "STOP":
            return
        self.follow_off_at = time.monotonic()
        for _ in range(3):
            self.follow_pub.publish(Bool(data=False))
        self.get_logger().warn(f"follow OFF ({why})")

    def pause_route(self, why):
        """Phase 3: the gateway runs no routes in v1."""
        self.get_logger().warn(f"route pause requested ({why}) -- no gateway route in v1")
