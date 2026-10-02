#!/usr/bin/env python3
"""Stationary yaw-sensor health check.

While the rover is lifted + still, sample every sensor that feeds yaw/angular
estimation and report stability (mean / std / peak-to-peak). Anything that
drifts or jumps here corrupts the turn-rate loop and the EKF heading.

  gyro (imu/data.angular_velocity)  -> yaw-RATE source (ATC_STR_RAT feedback)
  attitude (imu/data.orientation)   -> EKF fused yaw
  mag (imu/mag)                     -> compass heading (EKF yaw source, EK3_SRC1_YAW=1)
  compass_hdg                       -> FCU compass heading (deg)
Run with the UDP-only DDS profile.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=10)


def yaw_from_quat(x, y, z, w):
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


class Stat:
    def __init__(self):
        self.v = []
    def add(self, x):
        self.v.append(x)
    def report(self, unwrap_deg=False):
        if not self.v:
            return "NO DATA"
        v = self.v
        if unwrap_deg:  # unwrap heading so 359->1 isn't a 358 jump
            u = [v[0]]
            for k in v[1:]:
                d = k - u[-1]
                while d > 180: d -= 360
                while d < -180: d += 360
                u.append(u[-1] + d)
            v = u
        n = len(v)
        mean = sum(v) / n
        var = sum((x - mean) ** 2 for x in v) / n
        std = var ** 0.5
        return (f"n={n:4d}  mean={mean:+8.4f}  std={std:8.5f}  "
                f"min={min(v):+8.4f}  max={max(v):+8.4f}  p2p={max(v)-min(v):8.5f}")


class SensorHealth(Node):
    def __init__(self):
        super().__init__("sensor_health")
        self.gx, self.gy, self.gz = Stat(), Stat(), Stat()
        self.ax, self.ay, self.az = Stat(), Stat(), Stat()
        self.yaw = Stat()
        self.mx, self.my, self.mz = Stat(), Stat(), Stat()
        self.mhdg = Stat()
        self.chdg = Stat()
        self.n_imu = self.n_mag = self.n_comp = 0
        self.create_subscription(Imu, "/mavros/imu/data", self.imu_cb, BE)
        self.create_subscription(MagneticField, "/mavros/imu/mag", self.mag_cb, BE)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg",
                                 self.comp_cb, BE)

    def imu_cb(self, m):
        self.n_imu += 1
        self.gx.add(m.angular_velocity.x); self.gy.add(m.angular_velocity.y)
        self.gz.add(m.angular_velocity.z)
        self.ax.add(m.linear_acceleration.x); self.ay.add(m.linear_acceleration.y)
        self.az.add(m.linear_acceleration.z)
        q = m.orientation
        self.yaw.add(yaw_from_quat(q.x, q.y, q.z, q.w))

    def mag_cb(self, m):
        self.n_mag += 1
        bx, by, bz = m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z
        self.mx.add(bx); self.my.add(by); self.mz.add(bz)
        self.mhdg.add(math.degrees(math.atan2(-by, bx)))

    def comp_cb(self, m):
        self.n_comp += 1
        self.chdg.add(m.data)

    def report(self):
        print("\n================ STATIONARY YAW-SENSOR HEALTH ================")
        print(f"imu msgs={self.n_imu}  mag msgs={self.n_mag}  "
              f"compass msgs={self.n_comp}\n")
        print("-- GYRO (rad/s)  [yaw-rate loop feedback; should be ~0, low std] --")
        print(f"  gx  {self.gx.report()}")
        print(f"  gy  {self.gy.report()}")
        print(f"  gz  {self.gz.report()}   <-- YAW RATE")
        print("\n-- ACCEL (m/s^2)  [vibration proxy] --")
        print(f"  ax  {self.ax.report()}")
        print(f"  ay  {self.ay.report()}")
        print(f"  az  {self.az.report()}")
        print("\n-- EKF YAW (deg, unwrapped)  [fused heading; drift = bad] --")
        print(f"  yaw {self.yaw.report(unwrap_deg=True)}")
        print("\n-- MAGNETOMETER (Tesla) + derived mag heading --")
        print(f"  mx  {self.mx.report()}")
        print(f"  my  {self.my.report()}")
        print(f"  mz  {self.mz.report()}")
        print(f"  mag_hdg(deg) {self.mhdg.report(unwrap_deg=True)}")
        print("\n-- FCU COMPASS HEADING (deg, unwrapped) --")
        print(f"  compass_hdg {self.chdg.report(unwrap_deg=True)}")
        print("=============================================================")
        sys.stdout.flush()


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    n = SensorHealth()
    end = time.time() + dur
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.report()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
