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
import collections
import math
import time

from action_msgs.srv import CancelGoal
from mavros_msgs.msg import GPSRAW, RCIn, RCOut, State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks
from sensor_msgs.msg import BatteryState, CameraInfo, Imu, LaserScan, NavSatFix
from std_msgs.msg import Bool, String

from .readiness import Readiness
from .rover_state import pending_checks

WINDOW_S = 5.0             # rates and fractions are judged over this, as preflight's 5 s sample

# CPU (py-spy, 2026-10-12: 37 % of a core, most of it in rclpy's executor): by
# default rclpy gives every subscription and publisher a QoS-event handler, and
# every incoming message makes the executor rebuild its wait set with all of
# them. The gateway does not use QoS events, so it creates none.
QUIET_SUB = SubscriptionEventCallbacks(use_default_callbacks=False)
QUIET_PUB = PublisherEventCallbacks(use_default_callbacks=False)
SPEED_DEADBAND_MPS = 0.05  # a parked rover's /odometry/filtered shows a few mm/s

# topic, type, raw (no deserialisation: only the arrival counts). The rates each
# check needs are in readiness.py, from tools/preflight.py.
TOPICS = (
    ("/mavros/state", State, False),
    ("/mavros/local_position/odom", Odometry, False),
    ("/odometry/filtered", Odometry, False),
    ("/mavros/imu/data", Imu, True),
    ("/mavros/imu/data_raw", Imu, True),
    ("/mavros/rc/out", RCOut, False),
    ("/mavros/rc/in", RCIn, True),
    ("/mavros/gpsstatus/gps1/raw", GPSRAW, False),
    ("/mavros/global_position/global", NavSatFix, False),
    ("/mavros/battery", BatteryState, False),
    ("/gps/heading_imu", Imu, True),
    # Not /rslidar_points: ~1 MB x 10 Hz copied into Python only to be counted.
    # /scan is made from it (pointcloud_to_laserscan, nav2.launch.py), so /scan
    # at rate means the LiDAR is delivering.
    ("/scan", LaserScan, True),
    ("/d435/color/camera_info", CameraInfo, True),   # one per colour frame, ~1 KB
)


class Seen:
    """One topic: arrival times over the last WINDOW_S and the latest message."""

    def __init__(self):
        self.times = collections.deque()
        self.first = None
        self.last = None

    def add(self, msg, now):
        self.times.append(now)
        self.first = self.first or now
        self.last = msg
        while self.times[0] < now - WINDOW_S:
            self.times.popleft()

    def rate(self, now):
        """Hz over the window (or since first heard); 0 once it has gone quiet."""
        if self.first is None:
            return 0.0
        span = min(WINDOW_S, now - self.first)
        n = sum(1 for t in self.times if t >= now - WINDOW_S)
        return n / span if span > 0.5 else 0.0

    def age(self, now):
        return now - self.times[-1] if self.times else float("inf")


class Series:
    """Values with their times over the last WINDOW_S (e.g. heading valid or not)."""

    def __init__(self):
        self.items = collections.deque()

    def add(self, v, now):
        self.items.append((now, v))
        while self.items[0][0] < now - WINDOW_S:
            self.items.popleft()

    def values(self, now):
        return [v for t, v in self.items if t >= now - WINDOW_S]

FOLLOW_OFF_GRACE_S = 0.7   # ignore "following" from /follow/state this long after we turned it off
# After a release, ignore /estop true this long: our own 10 Hz stream, sent just
# before we heard the release, can still be in flight and must not re-latch.
# A real tools/estop.sh streams at 10 Hz, so it still latches on its next message.
RELEASE_GRACE_S = 0.5
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
        # position quality GOOD (good enough to drive a route) at or below this 1-sigma
        p("good_position_accuracy_m", 0.5)
        p("battery_low_percent", 30.0)
        p("battery_critical_percent", 15.0)
        # checks not yet heard stay PENDING (STARTING) this long while the stack comes up
        p("startup_grace_s", 120.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.param = g
        self.dry_run = bool(g("stop_dry_run"))
        self.on_estop_edge = None          # set by the gateway: called (in this thread) with True/False

        self.latched = False
        self.released_at = -1e9            # monotonic s of the last /estop false edge
        self.follow_state = None           # (text, monotonic s)
        self.follow_off_at = -1e9

        self.estop_pub = self.create_publisher(Bool, "/estop", 10, event_callbacks=QUIET_PUB)
        self.follow_pub = self.create_publisher(Bool, "/follow/enable", 10, event_callbacks=QUIET_PUB)
        self.create_subscription(Bool, "/estop", self.on_estop, 10, event_callbacks=QUIET_SUB)
        self.create_subscription(String, "/follow/state", self.on_follow_state, 10,
                                 event_callbacks=QUIET_SUB)
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.cancel_clis = [self.create_client(CancelGoal, f"/{a}/_action/cancel_goal")
                            for a in ("navigate_to_pose", "navigate_through_poses")]
        self.create_timer(0.1, self.stream_estop)

        # telemetry and readiness: one subscription per topic, shared by both
        self.seen = {t: Seen() for t, _, _ in TOPICS}
        self.gps_yaw_valid = Series()      # per gps1/raw: dual-antenna heading valid
        self.fcu = Series()                # per local_position/odom: (x, y, speed)
        self.rc_neutral = Series()         # per rc/out: both tracks at neutral
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        for topic, typ, raw in TOPICS:
            self.create_subscription(typ, topic, self._seen_cb(topic), be, raw=raw,
                                     event_callbacks=QUIET_SUB)
        self.checks = pending_checks()     # replaced whole once a second; read by the loop
        self.readiness = Readiness(self)
        self.create_timer(1.0, self.evaluate)

    # ---------------- in ----------------
    def on_estop(self, m):
        if m.data == self.latched:
            return
        now = time.monotonic()
        if m.data and now - self.released_at < RELEASE_GRACE_S:
            return                         # most likely our own stream, still in flight
        self.latched = m.data
        if not m.data:
            self.released_at = now
        self.get_logger().warn("/estop true: STOPPED latched" if m.data else
                               "/estop false: latch released (mode stays HOLD)")
        if self.on_estop_edge:
            self.on_estop_edge(m.data)

    def _seen_cb(self, topic):
        seen = self.seen[topic]

        def cb(msg):
            now = time.monotonic()
            seen.add(msg, now)
            if topic == "/mavros/gpsstatus/gps1/raw":
                self.gps_yaw_valid.add(msg.yaw not in (0, 65535), now)
            elif topic == "/mavros/local_position/odom":
                p, v = msg.pose.pose.position, msg.twist.twist.linear
                self.fcu.add((p.x, p.y, math.hypot(v.x, v.y)), now)
            elif topic == "/mavros/rc/out":
                ch = msg.channels
                self.rc_neutral.add(len(ch) > 2 and abs(ch[0] - 1500) <= 25
                                    and abs(ch[2] - 1500) <= 25, now)
        return cb

    def evaluate(self):
        self.checks = self.readiness.evaluate()

    def fresh(self, topic, max_age_s=2.0):
        """The topic's latest message if it is recent, else None."""
        s = self.seen[topic]
        return s.last if s.age(time.monotonic()) <= max_age_s else None

    def fill_state(self, st, checks):
        """Telemetry into a pb.RoverState. Absent means absent: no fix -> no position,
        heading invalid -> no heading, no real battery reading -> no battery.
        checks: CheckId -> (status, values), for the qualities."""
        from . import api_pb2 as pb
        ok = lambda cid: checks.get(cid, (None,))[0] == pb.CHECK_STATUS_PASS  # noqa: E731
        state = self.fresh("/mavros/state", 3.0)
        st.armed = bool(state and state.armed)

        fix, raw = self.fresh("/mavros/global_position/global"), self.fresh("/mavros/gpsstatus/gps1/raw")
        if fix is not None and raw is not None and raw.fix_type >= 2 \
                and math.isfinite(fix.latitude) and math.isfinite(fix.longitude):
            if raw.h_acc > 0:
                acc = raw.h_acc / 1000.0
            else:                          # no h_acc from this receiver: mavros' covariance
                acc = math.sqrt(max(fix.position_covariance[0], fix.position_covariance[4], 0.0))
            good = raw.fix_type >= 3 and 0 < acc <= self.param("good_position_accuracy_m") \
                and ok(pb.CHECK_POSITION)
            st.position.CopyFrom(pb.Position(
                latitude_deg=fix.latitude, longitude_deg=fix.longitude, horizontal_accuracy_m=acc,
                quality=pb.QUALITY_GOOD if good else pb.QUALITY_POOR))

        # dual-antenna heading straight from GPS_RAW_INT: cdeg clockwise from true
        # north, 0 and 65535 = unknown (gps_heading_imu republishes the same for EK3)
        if raw is not None and raw.yaw not in (0, 65535):
            st.heading.CopyFrom(pb.Heading(
                heading_deg=(raw.yaw / 100.0) % 360.0, accuracy_deg=raw.hdg_acc / 1e5,
                quality=pb.QUALITY_GOOD if ok(pb.CHECK_HEADING) else pb.QUALITY_POOR))

        odom = self.fresh("/odometry/filtered", 1.0)
        if odom is not None:
            v = odom.twist.twist.linear
            sp = math.hypot(v.x, v.y)
            st.speed_mps = sp if sp >= SPEED_DEADBAND_MPS else 0.0

        pct = self.battery_percent()
        if pct is not None:
            st.battery.CopyFrom(pb.Battery(percent=pct,
                                           voltage_v=self.seen["/mavros/battery"].last.voltage))

    def battery_percent(self):
        """0-100 from /mavros/battery, or None when the Cube has no real reading
        (no power module: percentage NaN or -0.01, voltage 0)."""
        b = self.fresh("/mavros/battery", 5.0)
        if b is None or not (b.voltage > 1.0) or not (0.0 <= b.percentage <= 1.0):
            return None
        return round(b.percentage * 100.0, 1)

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
            # Latched all the same (stream_estop publishes nothing in a dry run), so
            # tools/estop_clear.sh's /estop false is an edge and releases it.
            self.latched = True
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
