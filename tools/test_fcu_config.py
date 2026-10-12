#!/usr/bin/env python3
"""Offline test for fcu_config. NO ROVER, NO FCU.

    python3 tools/test_fcu_config.py

Runs in ROS_DOMAIN_ID 42 (set before rclpy is imported, inherited by the node it
starts). Plays mavros: /mavros/state, the setpoint_velocity parameter services
(mav_frame starts LOCAL_NED, as after every mavros restart) and cmd/command.
Checks fcu_config sets BODY_NED + every stream rate on connect, and again after
the FCU link drops and comes back.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "42"
import signal, subprocess, sys, threading, time  # noqa: E401,E402
import rclpy  # noqa: E402
from mavros_msgs.msg import State  # noqa: E402
from mavros_msgs.srv import CommandLong  # noqa: E402
from rcl_interfaces.msg import ParameterValue  # noqa: E402
from rcl_interfaces.srv import GetParameters, SetParameters  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "dagger_bringup"))
from dagger_bringup.fcu_config import STREAMS  # noqa: E402

SRC = ("export ROS_DOMAIN_ID=42 && source /opt/ros/humble/setup.bash && "
       "source /home/jetson/ros2_ws/install/setup.bash")


class FakeMavros(Node):
    def __init__(self):
        super().__init__("fake_mavros")
        self.frame, self.intervals, self.connected = "LOCAL_NED", [], False
        self.state_pub = self.create_publisher(State, "/mavros/state", 10)
        self.create_timer(0.5, lambda: self.state_pub.publish(State(connected=self.connected)))
        self.create_service(SetParameters, "/mavros/setpoint_velocity/set_parameters", self.on_set)
        self.create_service(GetParameters, "/mavros/setpoint_velocity/get_parameters", self.on_get)
        self.create_service(CommandLong, "/mavros/cmd/command", self.on_cmd)

    def on_set(self, req, res):
        self.frame = req.parameters[0].value.string_value
        return res

    def on_get(self, req, res):
        res.values = [ParameterValue(type=4, string_value=self.frame)]
        return res

    def on_cmd(self, req, res):
        self.intervals.append((int(req.param1), round(1e6 / req.param2, 3)))
        res.success = True
        return res


def wait(cond, timeout):
    end = time.monotonic() + timeout
    while not cond() and time.monotonic() < end:
        time.sleep(0.1)
    return cond()


def main():
    proc = subprocess.Popen(["bash", "-lc", f"{SRC} && exec ros2 run dagger_bringup fcu_config"],
                            stdout=open("/tmp/fcu_config_test.log", "w"), stderr=subprocess.STDOUT,
                            preexec_fn=os.setsid)
    rclpy.init()
    m = FakeMavros()
    ex = MultiThreadedExecutor()
    ex.add_node(m)
    threading.Thread(target=ex.spin, daemon=True).start()
    want = sorted((i, h) for i, h in STREAMS)
    ok = True
    try:
        time.sleep(2.0)
        check = lambda c, what: print(("ok    " if c else "FAIL  ") + what) or c  # noqa: E731
        ok &= check(m.frame == "LOCAL_NED" and not m.intervals, "nothing applied while the FCU is not connected")
        m.connected = True
        ok &= check(wait(lambda: m.frame == "BODY_NED" and len(m.intervals) >= len(STREAMS), 15),
                    "connect -> mav_frame BODY_NED")
        ok &= check(sorted(m.intervals) == want, f"every stream rate set once {sorted(m.intervals)}")
        m.connected, m.frame, m.intervals = False, "LOCAL_NED", []       # FCU reboot / mavros reconnect
        time.sleep(1.5)
        m.connected = True
        ok &= check(wait(lambda: m.frame == "BODY_NED" and len(m.intervals) >= len(STREAMS), 15),
                    "reconnect -> applied again")
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5)
        ex.shutdown()
        rclpy.try_shutdown()
    print("\nPASS" if ok else "\nFAIL (log: /tmp/fcu_config_test.log)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
