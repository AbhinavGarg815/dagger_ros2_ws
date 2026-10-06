#!/usr/bin/env python3
"""Dagger preflight: per-bring-up FCU config + health gate. Run before ANY Nav2 goal.

Run after T1 (drive_bringup) and T2 (nav2_odom_test) are up:
    python3 tools/preflight.py              # configure + check (5 s sample)
    python3 tools/preflight.py --no-config  # check only
    python3 tools/preflight.py -t 60        # 60 s sample -> stationary pose-noise baseline

NON-MOTION. It only (1) sets /mavros/setpoint_velocity mav_frame=BODY_NED (resets on
every mavros restart) and (2) raises MAVLink stream rates via MAV_CMD_SET_MESSAGE_INTERVAL
(not persistent across FCU reboot / mavros reconnect), then reads topics + TF.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, LaserScan
from std_msgs.msg import Bool, Float32, Float64
from mavros_msgs.msg import GPSRAW, State, RCOut, RCIn
from mavros_msgs.srv import CommandLong
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from lifecycle_msgs.srv import GetState

# (msg_id, Hz) -- LOCAL_POSITION_NED, ATTITUDE, RAW_IMU, SERVO_OUTPUT_RAW, GPS_RAW_INT,
# RC_CHANNELS (GPS_RAW_INT carries the dual-antenna heading fused by gps_heading_imu).
# RC_CHANNELS (65) feeds /mavros/rc/in, which mavros does not publish otherwise. Without
# it there is no way to tell pilot stick input from autonomous output: /mavros/rc/out is
# the servo command and looks identical either way. That ambiguity made a stop-distance
# measurement uninterpretable on 2026-09-25.
STREAMS = [(32, 20.0), (30, 10.0), (27, 10.0), (36, 25.0), (24, 5.0), (65, 10.0)]

# topic, type, min_rate_hz, raw (skip deserialization for big msgs)
TOPICS = [
    ("/mavros/local_position/odom", Odometry, 15.0, False),
    ("/odometry/filtered", Odometry, 15.0, False),
    ("/mavros/imu/data", Imu, 8.0, False),
    ("/mavros/imu/data_raw", Imu, 8.0, False),
    ("/mavros/rc/out", RCOut, 20.0, False),
    ("/mavros/rc/in", RCIn, 5.0, False),
    ("/mavros/gpsstatus/gps1/raw", GPSRAW, 4.0, False),
    # 3.6 Hz healthy; 0.4 Hz on 2026-10-03 when the GNSS heading was valid 8 % of the time
    # and EK3 (EK3_SRC1_YAW=2, GPS yaw only) diverged (Q67)
    ("/gps/heading_imu", Imu, 2.0, False),
    ("/mavros/global_position/compass_hdg", Float64, 1.0, False),
    ("/mavros/state", State, 0.5, False),
    ("/rslidar_points", PointCloud2, 8.0, True),
    # indoors (--indoor) the Cube's position, velocity and heading come from this (extnav_relay, Q80)
    ("/lidar_odom/odometry", Odometry, 8.0, False),
    ("/scan", LaserScan, 8.0, True),
]
NAV2_NODES = ["controller_server", "planner_server", "bt_navigator", "behavior_server"]
FIX = {0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Preflight(Node):
    def __init__(self):
        super().__init__("dagger_preflight")
        self.stats = {t: {"n": 0, "t0": None, "t1": None, "last": None} for t, *_ in TOPICS}
        self.odom_xy = []  # (x, y, yaw) from /odometry/filtered for noise baseline
        # Q67: judge the GNSS heading and the FCU estimator over the WHOLE window, not
        # from the last message (the heading was valid 8 % of the time and the last
        # message happened to be valid, so preflight passed with EK3 diverged).
        self.gps_yaw_valid = []   # per gps1/raw message
        self.fcu = []             # (x, y, body speed) per /mavros/local_position/odom
        self.rc_neutral = []      # per rc/out: both track outputs at neutral
        self.lat = []      # now - stamp for /odometry/filtered
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=50)
        for topic, typ, _, raw in TOPICS:
            self.create_subscription(typ, topic, self._mk_cb(topic, raw), be, raw=raw)
        # SLAM localization health (slam_geo_init; latched)
        self.slam_localized = None
        self.slam_score = None
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/slam/localized",
                                 lambda m: setattr(self, "slam_localized", m.data), latched)
        self.create_subscription(Float32, "/slam/match_score",
                                 lambda m: setattr(self, "slam_score", m.data), 10)
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

    def _mk_cb(self, topic, raw):
        def cb(msg):
            now = time.monotonic()
            s = self.stats[topic]
            s["n"] += 1
            s["t0"] = s["t0"] or now
            s["t1"] = now
            if not raw:
                s["last"] = msg
            if topic == "/mavros/gpsstatus/gps1/raw":
                self.gps_yaw_valid.append(msg.yaw not in (0, 65535))
            elif topic == "/mavros/local_position/odom":
                p, v = msg.pose.pose.position, msg.twist.twist.linear
                self.fcu.append((p.x, p.y, math.hypot(v.x, v.y)))
            elif topic == "/mavros/rc/out":
                ch = msg.channels
                self.rc_neutral.append(len(ch) > 2 and abs(ch[0] - 1500) <= 25
                                       and abs(ch[2] - 1500) <= 25)
            if topic == "/odometry/filtered":
                p = msg.pose.pose
                self.odom_xy.append((p.position.x, p.position.y, yaw_of(p.orientation)))
                st = Time.from_msg(msg.header.stamp)
                self.lat.append((self.get_clock().now() - st).nanoseconds * 1e-9)
        return cb

    def call(self, cli, req, timeout=3.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def configure(self):
        ok = True
        # SAFETY-CRITICAL: with mav_frame=LOCAL_NED the FCU reads Nav2's body-frame
        # twist in the EARTH frame and drives the wrong way. The set call can time out
        # right after a daemon/mavros restart (seen 2026-09-17: stayed LOCAL_NED), so
        # retry and VERIFY by reading the value back instead of trusting the set.
        set_cli = self.create_client(SetParameters, "/mavros/setpoint_velocity/set_parameters")
        get_cli = self.create_client(GetParameters, "/mavros/setpoint_velocity/get_parameters")
        frame = None
        for _ in range(4):
            req = SetParameters.Request()
            req.parameters = [Parameter(name="mav_frame", value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value="BODY_NED"))]
            self.call(set_cli, req, timeout=5.0)
            r = self.call(get_cli, GetParameters.Request(names=["mav_frame"]), timeout=5.0)
            frame = r.values[0].string_value if r and r.values else None
            if frame == "BODY_NED":
                break
        good = frame == "BODY_NED"
        ok &= good
        print(f"  [{'OK' if good else 'FAIL'}] setpoint_velocity mav_frame={frame} (verified by read-back)")
        cmd = self.create_client(CommandLong, "/mavros/cmd/command")
        for msg_id, hz in STREAMS:
            req = CommandLong.Request(command=511, param1=float(msg_id), param2=1e6 / hz)
            # 8 s, not the default 3 s: CommandLong round-trips over the 57600-baud
            # FCU link take 3-4 s normally and longer while the FCU is busy (e.g.
            # just after a rover reboot). At 3 s every call timed out and was
            # reported FAIL while actually succeeding -- measured success=True
            # result=0 in 4.20 s. A safety gate that cries wolf gets ignored.
            r = self.call(cmd, req, timeout=8.0)
            if not (r and r.success):
                r = self.call(cmd, req, timeout=8.0)
            good = bool(r and r.success)
            ok &= good
            print(f"  [{'OK' if good else 'FAIL'}] SET_MESSAGE_INTERVAL msg {msg_id} -> {hz:g} Hz")
        return ok

    def map_owners(self):
        """Who publishes map->odom: gps_map_odom, and/or slam_toolbox when it runs
        with transform_publish_period > 0 (localization; shadow mapping uses 0)."""
        names = set(self.get_node_names())
        owners = []
        if "gps_map_odom" in names:
            owners.append("gps_map_odom")
        if "slam_toolbox" in names:
            r = self.call(self.create_client(GetParameters, "/slam_toolbox/get_parameters"),
                          GetParameters.Request(names=["transform_publish_period", "mode"]), 3.0)
            period = r.values[0].double_value if r and r.values else None
            mode = r.values[1].string_value if r and len(r.values) > 1 else "?"
            if period is None or period > 0.0:
                owners.append(f"slam_toolbox ({mode})")
        return owners

    def lifecycle(self):
        out = {}
        for n in NAV2_NODES:
            r = self.call(self.create_client(GetState, f"/{n}/get_state"), GetState.Request(), 1.5)
            out[n] = r.current_state.label if r else "ABSENT"
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--sample", type=float, default=5.0, help="sample window, s")
    ap.add_argument("--no-config", action="store_true", help="skip FCU config, check only")
    ap.add_argument("--no-nav2", action="store_true", help="T1 only: skip Nav2/odom checks")
    ap.add_argument("--indoor", action="store_true",
                    help="indoor GUIDED (EK3 source set 2 = ExternalNav, Q80): GNSS fix/heading reported, not required; "
                         "/lidar_odom/odometry required instead")
    a = ap.parse_args()

    rclpy.init()
    n = Preflight()
    fails = []

    if not a.no_config:
        print("== configure FCU / mavros")
        if not n.configure():
            fails.append("config")
        # let the new stream rates take effect before measuring
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
        for s in n.stats.values():
            s.update(n=0, t0=None, t1=None)
        n.odom_xy.clear()
        n.lat.clear()
        n.gps_yaw_valid.clear()
        n.fcu.clear()
        n.rc_neutral.clear()

    print(f"== sampling {a.sample:g} s ...")
    end = time.monotonic() + a.sample
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

    print("== topic rates")
    for topic, _, min_hz, _ in TOPICS:
        if a.no_nav2 and topic in ("/odometry/filtered", "/scan", "/gps/heading_imu", "/lidar_odom/odometry"):
            continue
        if a.indoor and topic in ("/gps/heading_imu", "/mavros/gpsstatus/gps1/raw"):
            continue
        s = n.stats[topic]
        hz = (s["n"] - 1) / (s["t1"] - s["t0"]) if s["n"] > 1 and s["t1"] > s["t0"] else 0.0
        good = hz >= min_hz
        if not good:
            fails.append(topic)
        pubs = len(n.get_publishers_info_by_topic(topic))
        extra = f"  pubs={pubs}" + ("  <-- DUPLICATE PUBLISHER" if pubs > 1 and "odometry" in topic else "")
        if pubs > 1 and "odometry" in topic:
            fails.append(f"{topic} x{pubs}")
        print(f"  [{'OK' if good else 'FAIL'}] {topic:40s} {hz:6.1f} Hz (min {min_hz:g}){extra}")

    print("== FCU / GNSS")
    st = n.stats["/mavros/state"]["last"]
    if st:
        print(f"  connected={st.connected} armed={st.armed} mode={st.mode}")
        if not st.connected:
            fails.append("fcu-connected")
    else:
        fails.append("mavros/state")
        print("  [FAIL] no /mavros/state")
    g = n.stats["/mavros/gpsstatus/gps1/raw"]["last"]
    if g:
        yaw_ok = g.yaw not in (0, 65535)
        print(f"  fix={FIX.get(g.fix_type, g.fix_type)} sats={g.satellites_visible} "
              f"h_acc={g.h_acc / 1000:.2f} m  eph={g.eph / 100:.2f}  "
              f"gps_yaw={'%.1f deg' % (g.yaw / 100) if yaw_ok else 'UNAVAILABLE'}  "
              f"hdg_acc={g.hdg_acc / 1e5:.2f} deg")
        if g.fix_type < 3 and not a.indoor:
            fails.append("gps-fix")
        nv = len(n.gps_yaw_valid)
        frac = sum(n.gps_yaw_valid) / nv if nv else 0.0
        good = nv >= 5 and frac >= 0.9
        print(f"  [{'OK' if good else 'FAIL'}] GNSS heading valid in {100 * frac:.0f} % of {nv} messages "
              f"(min 90 %: EK3_SRC1_YAW=2 has no other yaw source)")
        if not good and not a.indoor:
            fails.append("gps-yaw")
        if a.indoor:
            print("  (--indoor: GNSS fix and heading reported only; the Cube navigates on ExternalNav)")
    elif not a.indoor:
        fails.append("gps1/raw")
    # FCU estimator sanity while parked (Q67: EK3 reported 20-260 m/s and its position
    # jumped 109 m in 10 s with the rover standing still). Healthy: < 0.02 m/s, < 0.2 m.
    if len(n.fcu) >= 10 and len(n.rc_neutral) >= 3 and all(n.rc_neutral):
        sp = max(f[2] for f in n.fcu)
        moved = math.hypot(n.fcu[-1][0] - n.fcu[0][0], n.fcu[-1][1] - n.fcu[0][1])
        good = sp <= 0.5 and moved <= 2.0
        print(f"  [{'OK' if good else 'FAIL'}] FCU estimator while parked: speed max {sp:.2f} m/s "
              f"(max 0.5), position moved {moved:.2f} m in {a.sample:g} s (max 2.0)")
        if not good:
            fails.append("fcu-estimator")
    elif n.fcu:
        print("  [--] FCU estimator check skipped: tracks not at neutral (rover being driven?)")
    else:
        fails.append("fcu-estimator")
        print("  [FAIL] no /mavros/local_position/odom")
    c = n.stats["/mavros/global_position/compass_hdg"]["last"]
    if c:
        print(f"  compass_hdg (EK3 yaw) = {c.data:.1f} deg")

    if not a.no_nav2:
        print("== TF / odom")
        try:
            tr = n.tf_buf.lookup_transform("odom", "base_footprint", Time())
            age = (n.get_clock().now() - Time.from_msg(tr.header.stamp)).nanoseconds * 1e-9
            good = age < 0.3
            print(f"  [{'OK' if good else 'FAIL'}] odom->base_footprint age {age * 1000:.0f} ms")
            if not good:
                fails.append("tf-age")
        except Exception as e:  # noqa: BLE001
            fails.append("tf")
            print(f"  [FAIL] odom->base_footprint: {e}")
        try:
            n.tf_buf.lookup_transform("base_footprint", "rslidar", Time())
            print("  [OK] base_footprint->rslidar")
        except Exception as e:  # noqa: BLE001
            fails.append("tf-lidar")
            print(f"  [FAIL] base_footprint->rslidar: {e}")
        if n.lat:
            lat = sorted(n.lat)
            print(f"  /odometry/filtered stamp latency: median {lat[len(lat) // 2] * 1000:.0f} ms, "
                  f"p95 {lat[int(len(lat) * 0.95)] * 1000:.0f} ms")
        if len(n.odom_xy) > 10:
            xs, ys, yaws = zip(*n.odom_xy)
            mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
            sd = math.sqrt(sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys)) / len(xs))
            span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            yspan = math.degrees(max(yaws) - min(yaws))
            print(f"  pose @ ({mx:.2f}, {my:.2f}) yaw {math.degrees(yaws[-1]):.1f} deg")
            print(f"  IF STATIONARY -> position noise: 1-sigma {sd:.3f} m, span {span:.3f} m; "
                  f"yaw span {yspan:.2f} deg over {a.sample:g} s")

        # odom must have exactly ONE parent: gps_map_odom (T2 map_source:=gps) or
        # slam_toolbox localization (T5 mode:=localization + T2 map_source:=slam).
        print("== map->odom")
        owners = n.map_owners()
        good = len(owners) == 1
        print(f"  [{'OK' if good else 'FAIL'}] owner: {', '.join(owners) or 'NONE'}"
              + ("" if good else "  <-- need exactly one"))
        if not good:
            fails.append("map-owner")
        try:
            n.tf_buf.lookup_transform("map", "base_footprint", Time())
            print("  [OK] map->base_footprint")
        except Exception as e:  # noqa: BLE001
            fails.append("tf-map")
            print(f"  [FAIL] map->base_footprint: {e}")
        if any(o.startswith("slam_toolbox") for o in owners):
            good = n.slam_localized is True
            score = f"{n.slam_score:.3f} m" if n.slam_score is not None else "none"
            print(f"  [{'OK' if good else 'FAIL'}] SLAM localized={n.slam_localized} "
                  f"(scan-to-map score {score}; locked ~0.03-0.10, lost >= 0.35)")
            if not good:
                fails.append("slam-localized")

        print("== Nav2 lifecycle")
        for node, label in n.lifecycle().items():
            good = label == "active"
            if not good:
                fails.append(node)
            print(f"  [{'OK' if good else 'FAIL'}] {node:18s} {label}")

    print("\n" + ("PREFLIGHT PASS" if not fails else "PREFLIGHT FAIL: " + ", ".join(fails)))
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if not fails else 1)


if __name__ == "__main__":
    main()
