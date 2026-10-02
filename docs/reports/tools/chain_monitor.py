#!/usr/bin/env python3
"""Aligned capture of the whole GUIDED command chain on one timeline.

teleop -> twist_mux -> cmd_vel_safety -> FCU -> PWM -> IMU

Every row is a 10Hz snapshot of the LATEST value on each link, plus the AGE of
that value. The age columns are the point: they show when a command goes stale
(e.g. cmd_vel_safety's 0.5s watchdog zeroing the setpoint between keytaps).

Run with the UDP-only DDS profile, since MAVROS runs as 'jetson' and this runs
as 'nvidia' (see udp_only.xml):

  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/dagger_reports/tools/udp_only.xml
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  python3 chain_monitor.py /path/to/out.csv
"""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCOut, State, NavControllerOutput

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=10)
REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST, depth=10)


class ChainMonitor(Node):
    def __init__(self, path):
        super().__init__("chain_monitor")
        self.t0 = time.time()
        self.v = {}

        def stash(name):
            def cb(msg):
                if isinstance(msg, Twist):
                    self.v[name] = ((msg.linear.x, msg.angular.z), time.time())
                elif isinstance(msg, RCOut):
                    ch = list(msg.channels) + [0] * 8
                    self.v[name] = ((ch[0], ch[2]), time.time())
                elif isinstance(msg, Imu):
                    self.v[name] = ((msg.angular_velocity.z, 0.0), time.time())
                elif isinstance(msg, State):
                    self.v[name] = ((msg.mode, 1 if msg.armed else 0), time.time())
                elif isinstance(msg, NavControllerOutput):
                    # nav_bearing/target_bearing are non-zero only if a HEADING
                    # loop is driving steering -- the tell for which GUIDED
                    # submode Rover picked (heading-and-speed vs turn-rate).
                    self.v[name] = ((msg.nav_bearing, msg.target_bearing), time.time())
                elif isinstance(msg, Odometry):
                    q = msg.pose.pose.orientation
                    yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
                    self.v[name] = ((yaw, 0.0), time.time())
            return cb

        self.create_subscription(Twist, "/cmd_vel_teleop", stash("teleop"), REL)
        self.create_subscription(Twist, "/cmd_vel_out", stash("mux"), REL)
        self.create_subscription(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", stash("setpoint"), REL)
        self.create_subscription(RCOut, "/mavros/rc/out", stash("pwm"), BE)
        self.create_subscription(Imu, "/mavros/imu/data", stash("imu"), BE)
        self.create_subscription(State, "/mavros/state", stash("state"), BE)
        self.create_subscription(
            NavControllerOutput, "/mavros/nav_controller_output/output", stash("nav"), REL)
        self.create_subscription(Odometry, "/odometry/filtered", stash("odom"), BE)

        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "mode", "armed",
                         "teleop_lin", "teleop_ang", "teleop_age",
                         "mux_lin", "mux_ang", "mux_age",
                         "sp_lin", "sp_ang", "sp_age",
                         "ch1_steer", "ch3_thr", "imu_gz",
                         "nav_bearing", "target_bearing", "odom_yaw"])
        # 50Hz: /mavros/rc/out is raised to 25Hz via MAV_CMD_SET_MESSAGE_INTERVAL
        # (msg 36 @ 40000us). Sampling below 2x that aliases a fast control
        # oscillation into a fake slow square wave -- which cost us several runs.
        self.create_timer(0.02, self.tick)

    def get(self, name, default=(0.0, 0.0)):
        val, ts = self.v.get(name, (default, 0.0))
        age = (time.time() - ts) if ts else -1.0
        return val, age

    def tick(self):
        now = round(time.time() - self.t0, 2)
        st, _ = self.get("state", ("?", 0))
        tl, tage = self.get("teleop")
        mx, mage = self.get("mux")
        sp, sage = self.get("setpoint")
        pw, _ = self.get("pwm", (0, 0))
        im, _ = self.get("imu")
        nv, _ = self.get("nav", (0, 0))
        od, _ = self.get("odom", (0.0, 0.0))
        self.w.writerow([now, st[0], st[1],
                         round(tl[0], 3), round(tl[1], 3), round(tage, 2),
                         round(mx[0], 3), round(mx[1], 3), round(mage, 2),
                         round(sp[0], 4), round(sp[1], 4), round(sage, 2),
                         pw[0], pw[1], round(im[0], 3),
                         nv[0], nv[1], round(od[0], 2)])
        self.f.flush()


def main():
    rclpy.init()
    n = ChainMonitor(sys.argv[1])
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.f.close()


if __name__ == "__main__":
    main()
