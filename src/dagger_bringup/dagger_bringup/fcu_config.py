#!/usr/bin/env python3
"""Per-connection FCU/mavros configuration, re-applied every time mavros (re)connects.

What tools/preflight.py's configure step does, moved into bring-up (T1) so a rover
started by systemd with nobody logged in is configured too. NON-MOTION:
  1. /mavros/setpoint_velocity mav_frame = BODY_NED, verified by read-back.
     SAFETY-CRITICAL: with LOCAL_NED the FCU reads Nav2's body-frame twist in the
     EARTH frame and drives the wrong way. Resets on every mavros restart.
  2. MAVLink stream rates via MAV_CMD_SET_MESSAGE_INTERVAL (lost on FCU reboot or
     mavros reconnect). RC_CHANNELS feeds /mavros/rc/in, which is otherwise absent.
Triggered by /mavros/state connected going true (startup, FCU reboot, link back).
preflight.py imports STREAMS from here; its configure step still works by hand.
"""
import threading
import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# (msg_id, Hz) -- LOCAL_POSITION_NED, ATTITUDE, RAW_IMU, SERVO_OUTPUT_RAW, GPS_RAW_INT,
# RC_CHANNELS (GPS_RAW_INT carries the dual-antenna heading fused by gps_heading_imu).
STREAMS = [(32, 20.0), (30, 10.0), (27, 10.0), (36, 25.0), (24, 5.0), (65, 10.0)]


class FcuConfig(Node):

    def __init__(self):
        super().__init__("fcu_config")
        self.set_cli = self.create_client(SetParameters, "/mavros/setpoint_velocity/set_parameters")
        self.get_cli = self.create_client(GetParameters, "/mavros/setpoint_velocity/get_parameters")
        self.cmd_cli = self.create_client(CommandLong, "/mavros/cmd/command")
        self.connected = False
        self.wake = threading.Event()
        self.create_subscription(State, "/mavros/state", self.on_state, 10)
        threading.Thread(target=self.worker, daemon=True).start()

    def on_state(self, m):
        if m.connected and not self.connected:
            self.get_logger().info("FCU connected: applying mav_frame + stream rates")
            self.wake.set()
        self.connected = m.connected

    def call(self, cli, req, timeout):
        """Blocking call from the worker thread; the executor completes the future."""
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def worker(self):
        while rclpy.ok():
            self.wake.wait()
            self.wake.clear()
            # mavros' plugins and the FCU link may still be settling: keep at it
            for _ in range(6):
                if self.apply() or not self.connected:
                    break
                time.sleep(5.0)

    def apply(self):
        frame = None
        for _ in range(4):
            req = SetParameters.Request(parameters=[Parameter(name="mav_frame", value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value="BODY_NED"))])
            self.call(self.set_cli, req, 5.0)
            r = self.call(self.get_cli, GetParameters.Request(names=["mav_frame"]), 5.0)
            frame = r.values[0].string_value if r and r.values else None
            if frame == "BODY_NED":
                break
        ok = frame == "BODY_NED"
        (self.get_logger().info if ok else self.get_logger().error)(
            f"setpoint_velocity mav_frame={frame} (verified by read-back)")
        failed = []
        for msg_id, hz in STREAMS:
            # 8 s: CommandLong over the 57600-baud link takes 3-4 s, longer when busy
            req = CommandLong.Request(command=511, param1=float(msg_id), param2=1e6 / hz)
            r = self.call(self.cmd_cli, req, 8.0)
            if not (r and r.success):
                r = self.call(self.cmd_cli, req, 8.0)
            if not (r and r.success):
                failed.append(msg_id)
        if failed:
            self.get_logger().error(f"SET_MESSAGE_INTERVAL failed for msg {failed}")
        else:
            self.get_logger().info(f"stream rates set: {', '.join(f'{m}@{h:g}Hz' for m, h in STREAMS)}")
        return ok and not failed


def main():
    rclpy.init()
    node = FcuConfig()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
