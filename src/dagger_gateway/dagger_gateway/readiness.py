"""tools/preflight.py's checks, run continuously, mapped onto the app's nine CheckIds.

Evaluated once a second in the rclpy thread over the last 5 s (preflight's sample),
from ros_io's shared subscriptions. Same thresholds as preflight -- change them
together. Preflight's CONFIGURATION (stream rates, mav_frame) is not a check: it
belongs in bring-up, re-applied after every mavros restart; until it has run,
/mavros/rc/in and gps1/raw are missing and this honestly says NOT_READY.

PENDING: a check whose inputs have not all been heard yet, while the gateway is
younger than startup_grace_s (the stack is still coming up under systemd). After
that, missing means FAIL.

Not yet covered (preflight has them): exactly one map->odom owner; --indoor
(ExtNav, RC10 middle), where GNSS position and heading are not required.
"""
import math
import time

import tf2_ros
from lifecycle_msgs.srv import GetState
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool

from . import api_pb2 as pb

PASS, WARN, FAIL, PENDING = (pb.CHECK_STATUS_PASS, pb.CHECK_STATUS_WARN, pb.CHECK_STATUS_FAIL,
                             pb.CHECK_STATUS_PENDING)
NAV2_NODES = ("controller_server", "planner_server", "bt_navigator", "behavior_server")

# CheckId -> (topic, min Hz) -- preflight.TOPICS
RATES = {
    pb.CHECK_FLIGHT_CONTROLLER: (("/mavros/local_position/odom", 15.0), ("/mavros/imu/data", 8.0),
                                 ("/mavros/imu/data_raw", 8.0), ("/mavros/rc/out", 20.0),
                                 ("/mavros/state", 0.5)),
    pb.CHECK_REMOTE_CONTROL: (("/mavros/rc/in", 5.0),),
    pb.CHECK_POSITION: (("/mavros/gpsstatus/gps1/raw", 4.0),),
    # 3.6 Hz healthy; 0.4 Hz when the heading was valid 8 % of the time and EK3 diverged (Q67)
    pb.CHECK_HEADING: (("/mavros/gpsstatus/gps1/raw", 4.0), ("/gps/heading_imu", 2.0)),
    pb.CHECK_OBSTACLE_SENSOR: (("/rslidar_points", 8.0), ("/scan", 8.0)),
    pb.CHECK_LOCALISATION: (("/odometry/filtered", 15.0),),
    pb.CHECK_BATTERY: (("/mavros/battery", 0.0),),
    pb.CHECK_CAMERA: (("/d435/color/camera_info", 10.0),),   # 30 fps profile
}


class Readiness:

    def __init__(self, io):
        self.io = io
        self.t0 = time.monotonic()
        self.grace_s = float(io.param("startup_grace_s"))
        self.estimator_ok = None           # last verdict while the tracks were at neutral
        self.lifecycle = {}                # node -> label ("ABSENT" when no answer)
        self.slam_localized = None
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        io.create_subscription(Bool, "/slam/localized",
                               lambda m: setattr(self, "slam_localized", m.data), latched)
        self.tf = tf2_ros.Buffer(cache_time=Duration(seconds=5))
        self.tf_listener = tf2_ros.TransformListener(self.tf, io)
        self.lc_clients = {n: io.create_client(GetState, f"/{n}/get_state") for n in NAV2_NODES}
        io.create_timer(2.0, self.poll_lifecycle)

    def poll_lifecycle(self):
        for n, cli in self.lc_clients.items():
            if not cli.service_is_ready():
                self.lifecycle[n] = "ABSENT"
                continue
            fut = cli.call_async(GetState.Request())
            fut.add_done_callback(lambda f, n=n: self.lifecycle.__setitem__(
                n, f.result().current_state.label if f.result() else "ABSENT"))

    # ---------------- helpers ----------------
    def rates_ok(self, cid, now):
        return all(self.io.seen[t].rate(now) >= hz and self.io.seen[t].age(now) < 2.0
                   for t, hz in RATES.get(cid, ()))

    def heard(self, cid):
        if cid == pb.CHECK_NAVIGATION:
            return any(label != "ABSENT" for label in self.lifecycle.values())
        return all(self.io.seen[t].first is not None for t, _ in RATES.get(cid, ()))

    def tf_age(self, parent, child):
        try:
            tr = self.tf.lookup_transform(parent, child, Time())
        except tf2_ros.TransformException:
            return None
        return (self.io.get_clock().now() - Time.from_msg(tr.header.stamp)).nanoseconds * 1e-9

    # ---------------- the nine ----------------
    def evaluate(self):
        """CheckId -> (CheckStatus, values)."""
        now = time.monotonic()
        starting = now - self.t0 < self.grace_s
        out = {}
        for cid, fn in ((pb.CHECK_FLIGHT_CONTROLLER, self.flight_controller),
                        (pb.CHECK_REMOTE_CONTROL, self.remote_control),
                        (pb.CHECK_POSITION, self.position),
                        (pb.CHECK_HEADING, self.heading),
                        (pb.CHECK_OBSTACLE_SENSOR, self.obstacle_sensor),
                        (pb.CHECK_LOCALISATION, self.localisation),
                        (pb.CHECK_NAVIGATION, self.navigation),
                        (pb.CHECK_BATTERY, self.battery),
                        (pb.CHECK_CAMERA, self.camera)):
            status, values = fn(now)
            if starting and status != PASS and not self.heard(cid):
                status = PENDING
            out[cid] = (status, values)
        return out

    def flight_controller(self, now):
        st = self.io.fresh("/mavros/state", 3.0)
        return (PASS if st is not None and st.connected and self.rates_ok(pb.CHECK_FLIGHT_CONTROLLER, now)
                else FAIL), {}

    def remote_control(self, now):
        return (PASS if self.rates_ok(pb.CHECK_REMOTE_CONTROL, now) else FAIL), {}

    def position(self, now):
        raw = self.io.fresh("/mavros/gpsstatus/gps1/raw")
        fix_ok = raw is not None and raw.fix_type >= 3 and self.rates_ok(pb.CHECK_POSITION, now)
        # FCU estimator sanity while parked (Q67: EK3 at 20-260 m/s, position jumping
        # 109 m with the rover still). Judged only with both tracks at neutral; while
        # driving, the last parked verdict stands.
        fcu, neutral = self.io.fcu.values(now), self.io.rc_neutral.values(now)
        if len(fcu) >= 10 and len(neutral) >= 3 and all(neutral):
            sp = max(f[2] for f in fcu)
            moved = math.hypot(fcu[-1][0] - fcu[0][0], fcu[-1][1] - fcu[0][1])
            self.estimator_ok = sp <= 0.5 and moved <= 2.0
        elif not fcu:
            self.estimator_ok = None
        return (PASS if fix_ok and self.estimator_ok else FAIL), {}

    def heading(self, now):
        v = self.io.gps_yaw_valid.values(now)
        frac = sum(v) / len(v) if v else 0.0
        good = len(v) >= 5 and frac >= 0.9 and self.rates_ok(pb.CHECK_HEADING, now)
        return (PASS if good else FAIL), {"valid_percent": round(100.0 * frac)}

    def obstacle_sensor(self, now):
        return (PASS if self.rates_ok(pb.CHECK_OBSTACLE_SENSOR, now) else FAIL), {}

    def localisation(self, now):
        odom_age = self.tf_age("odom", "base_footprint")
        good = (self.rates_ok(pb.CHECK_LOCALISATION, now)
                and odom_age is not None and odom_age < 0.3
                and self.tf_age("base_footprint", "rslidar") is not None
                and self.tf_age("map", "base_footprint") is not None
                and self.slam_localized is not False)      # only when SLAM says
        return (PASS if good else FAIL), {}

    def navigation(self, now):
        return (PASS if all(self.lifecycle.get(n) == "active" for n in NAV2_NODES) else FAIL), {}

    def battery(self, now):
        pct = self.io.battery_percent()
        if pct is None:
            return WARN, {}                # cannot measure: say so, block nothing
        values = {"percent": pct}
        if pct < float(self.io.param("battery_critical_percent")):
            return FAIL, values
        if pct < float(self.io.param("battery_low_percent")):
            return WARN, values
        return PASS, values

    def camera(self, now):
        return (PASS if self.rates_ok(pb.CHECK_CAMERA, now) else FAIL), {}
