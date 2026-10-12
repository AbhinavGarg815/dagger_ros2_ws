#!/usr/bin/env python3
"""The gateway against fake rover topics. NO ROVER, NO FCU.

    python3 src/dagger_gateway/test/test_gateway_ros.py      (after colcon build)

Runs in ROS_DOMAIN_ID 42 (set before rclpy is imported, inherited by the gateway
it starts), so it cannot reach a live stack. Starts `ros2 run dagger_gateway
gateway` on port 18090 WITHOUT --stop-dry-run, plays the topics and services the
stack would provide (mavros, odometry, Nav2 cancel, follow_controller), connects
as the app, and checks what the gateway reports and does.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "42"
import asyncio  # noqa: E402
import math  # noqa: E402
import signal  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402

import rclpy  # noqa: E402
import tornado.websocket  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from mavros_msgs.msg import GPSRAW, RCIn, RCOut, State  # noqa: E402
from mavros_msgs.srv import SetMode  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import BatteryState, CameraInfo, Imu, LaserScan, NavSatFix, PointCloud2  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway import api_pb2 as pb  # noqa: E402

PORT = 18090
URL = f"ws://127.0.0.1:{PORT}/api"
SRC = ("export ROS_DOMAIN_ID=42 && source /opt/ros/humble/setup.bash && "
       "source /home/jetson/ros2_ws/install/setup.bash")
LOG = "/tmp/gateway_test.log"


class FakeRover(Node):
    """The topics and services of a parked rover, at preflight's rates."""

    def __init__(self):
        super().__init__("fake_rover")
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                        depth=10)
        pub = lambda t, typ: self.create_publisher(typ, t, be)  # noqa: E731
        self.pubs = {
            "state": pub("/mavros/state", State),
            "lodom": pub("/mavros/local_position/odom", Odometry),
            "odom": pub("/odometry/filtered", Odometry),
            "imu": pub("/mavros/imu/data", Imu), "imu_raw": pub("/mavros/imu/data_raw", Imu),
            "rcout": pub("/mavros/rc/out", RCOut), "rcin": pub("/mavros/rc/in", RCIn),
            "raw": pub("/mavros/gpsstatus/gps1/raw", GPSRAW),
            "fix": pub("/mavros/global_position/global", NavSatFix),
            "batt": pub("/mavros/battery", BatteryState),
            "hdg": pub("/gps/heading_imu", Imu),
            "points": pub("/rslidar_points", PointCloud2), "scan": pub("/scan", LaserScan),
            "caminfo": pub("/d435/color/camera_info", CameraInfo),
        }
        self.follow_state_pub = self.create_publisher(String, "/follow/state", 10)
        self.estop_pub = self.create_publisher(Bool, "/estop", 10)
        # what the scene plays; tests change these
        self.armed = True
        self.speed = 0.003                 # parked: odometry noise
        self.gps_yaw = 9050                # cdeg; 65535 = invalid
        self.fix_type = 3
        self.h_acc_mm = 3200
        self.battery = (float("nan"), 0.0)  # (fraction, volts): no power module
        self.follow_text = None            # what follow_controller says, None = not running
        self.quiet = set()                 # topics that stopped
        # what the gateway did
        self.estop_seen, self.follow_enable_seen = [], []
        self.modes, self.cancels = [], []
        self.create_subscription(Bool, "/estop", lambda m: self.estop_seen.append((time.monotonic(), m.data)), 10)
        self.create_subscription(Bool, "/follow/enable",
                                 lambda m: self.follow_enable_seen.append((time.monotonic(), m.data)), 10)
        self.create_service(SetMode, "/mavros/set_mode", self.on_mode)
        for a in ("navigate_to_pose", "navigate_through_poses"):
            self.create_service(CancelGoal, f"/{a}/_action/cancel_goal",
                                lambda req, res, a=a: (self.cancels.append(a), res)[1])
        self.n = 0
        self.create_timer(0.04, self.tick)            # 25 Hz base

    def on_mode(self, req, res):
        self.modes.append(req.custom_mode)
        res.mode_sent = True
        return res

    def tick(self):
        self.n += 1
        every = lambda k: self.n % k == 0  # noqa: E731
        stamp = self.get_clock().now().to_msg()
        p = self.pubs
        send = lambda key, msg: key not in self.quiet and p[key].publish(msg)  # noqa: E731
        o = Odometry()
        o.header.stamp = stamp
        o.twist.twist.linear.x = self.speed
        send("lodom", o)
        send("odom", o)
        send("rcout", RCOut(channels=[1500, 1500, 1500, 1500]))
        if every(2):
            send("imu", Imu())
            send("imu_raw", Imu())
            send("points", PointCloud2())
            send("scan", LaserScan())
            send("caminfo", CameraInfo())
            send("rcin", RCIn(channels=[1500] * 8))
        if every(5):
            send("raw", GPSRAW(fix_type=self.fix_type, h_acc=self.h_acc_mm, yaw=self.gps_yaw,
                               hdg_acc=150000, satellites_visible=20))
            f = NavSatFix(latitude=28.5, longitude=77.2)
            send("fix", f)
            if self.gps_yaw != 65535:
                send("hdg", Imu())
        if every(10):
            send("state", State(connected=True, armed=self.armed, mode="HOLD"))
            frac, volts = self.battery
            send("batt", BatteryState(percentage=frac, voltage=volts))
        if every(12) and self.follow_text is not None:
            self.follow_state_pub.publish(String(data=self.follow_text))


class App:
    """The app's side: handshake, heartbeats (optional), and an inbox."""

    def __init__(self, conn):
        self.conn, self.inbox, self.beat = conn, asyncio.Queue(), None
        self.rover_id = ""
        self.reader = asyncio.ensure_future(self._read())

    @classmethod
    async def connect(cls):
        return cls(await tornado.websocket.websocket_connect(URL))

    async def _read(self):
        while True:
            raw = await self.conn.read_message()
            if raw is None:
                return
            env = pb.Envelope()
            env.ParseFromString(raw)
            await self.inbox.put(env)

    async def send(self, **body):
        env = pb.Envelope(rover_id=self.rover_id, sent_at_ms=int(time.time() * 1000), **body)
        await self.conn.write_message(env.SerializeToString(), binary=True)

    async def receive(self, kind, timeout=2.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                env = await asyncio.wait_for(self.inbox.get(), end - time.monotonic())
            except asyncio.TimeoutError:
                return None
            if env.WhichOneof("body") == kind:
                return env
        return None

    async def latest_state(self, settle=0.5):
        """The newest state after `settle` s (states before it are stale)."""
        await asyncio.sleep(settle)
        last = None
        while not self.inbox.empty():
            env = self.inbox.get_nowait()
            if env.WhichOneof("body") == "state":
                last = env.state
        return last or (await self.receive("state")).state

    async def handshake(self, heartbeats=True):
        await self.send(hello=pb.Hello(protocol_version=pb.PROTOCOL_VERSION_1, app_version="test"))
        reply = await self.receive("hello_reply")
        assert reply and reply.hello_reply.accepted
        self.rover_id = reply.hello_reply.rover_id
        if heartbeats:
            self.start_beats()

    def start_beats(self):
        async def beat():
            seq = 0
            while True:
                seq += 1
                await self.send(heartbeat=pb.Heartbeat(seq=seq, sent_time_ms=int(time.monotonic() * 1000)))
                await asyncio.sleep(0.25)
        self.beat = asyncio.ensure_future(beat())

    def stop_beats(self):
        if self.beat:
            self.beat.cancel()
            self.beat = None

    def close(self):
        self.stop_beats()
        self.reader.cancel()
        self.conn.close()


PASSED = []


def check(cond, what):
    if not cond:
        print(f"FAIL  {what}", flush=True)
        raise SystemExit(1)
    PASSED.append(what)
    print(f"ok    {what}", flush=True)


async def scene_telemetry(rover):
    app = await App.connect()
    await app.handshake()
    st = await app.latest_state(1.0)
    check(st.armed, "armed from /mavros/state")
    check(st.HasField("position") and abs(st.position.latitude_deg - 28.5) < 1e-9
          and abs(st.position.horizontal_accuracy_m - 3.2) < 1e-6,
          f"position from global + gps1/raw h_acc ({st.position.horizontal_accuracy_m:.1f} m)")
    check(st.position.quality == pb.QUALITY_POOR, "3.2 m is POOR (not good enough for a route)")
    check(st.HasField("heading") and abs(st.heading.heading_deg - 90.5) < 1e-4
          and abs(st.heading.accuracy_deg - 1.5) < 1e-4, "heading from the dual-antenna yaw")
    check(st.speed_mps == 0.0, "parked odometry noise reads as 0 m/s")
    check(not st.HasField("battery"), "no power-module reading -> no battery (never a made-up value)")

    rover.speed, rover.battery, rover.gps_yaw = 0.42, (0.76, 24.3), 65535
    rover.fix_type = 1
    st = await app.latest_state(1.5)
    check(abs(st.speed_mps - 0.42) < 1e-4, "speed from /odometry/filtered")
    check(st.HasField("battery") and abs(st.battery.percent - 76.0) < 1e-3
          and abs(st.battery.voltage_v - 24.3) < 1e-3, "battery when the Cube measures it")
    check(not st.HasField("heading"), "invalid GNSS heading -> no heading")
    check(not st.HasField("position"), "no fix -> no position")
    rover.speed, rover.gps_yaw, rover.fix_type = 0.003, 9050, 3
    app.close()


async def scenes(rover):
    await scene_telemetry(rover)


def main():
    gw = subprocess.Popen(["bash", "-lc", f"{SRC} && exec ros2 run dagger_gateway gateway "
                           f"--ros-args -p port:={PORT}"],
                          stdout=open(LOG, "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    rclpy.init()
    rover = FakeRover()
    ex = MultiThreadedExecutor()
    ex.add_node(rover)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        async def run():
            for _ in range(100):           # wait for the gateway to listen
                try:
                    (await tornado.websocket.websocket_connect(URL)).close()
                    break
                except OSError:
                    await asyncio.sleep(0.1)
            await scenes(rover)
        asyncio.run(run())
    finally:
        os.killpg(gw.pid, signal.SIGTERM)
        gw.wait(timeout=5)
        ex.shutdown()
        rclpy.try_shutdown()
    print(f"\n{len(PASSED)} checks passed (gateway log: {LOG})")


if __name__ == "__main__":
    main()
