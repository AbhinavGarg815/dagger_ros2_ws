#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from std_msgs.msg import UInt32, Bool


RESET = "\033[0m"
BOLD = "\033[1m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
CYAN = "\033[36m"


class SystemReady(Node):

    def __init__(self):
        super().__init__("system_ready")

        self.declare_parameter("sensor_timeout", 3.0)
        self.declare_parameter("min_satellites", 6)
        self.sensor_timeout = float(self.get_parameter("sensor_timeout").value)
        self.min_satellites = int(self.get_parameter("min_satellites").value)

        self.fcu_connected = False
        self.gps_fixed = False
        self.satellites = 0
        self.last_lidar = None
        self.last_imu = None

        self.all_ready = False
        self.gps_announced = False
        self.last_status_log = self.get_clock().now()

        self.ready_pub = self.create_publisher(Bool, "/system_ready", 1)

        self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/raw/fix", self.fix_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            UInt32, "/mavros/global_position/raw/satellites", self.sat_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "/rslidar_points", self.lidar_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, "/mavros/imu/data", self.imu_cb, qos_profile_sensor_data
        )

        self.timer = self.create_timer(1.0, self.check)
        self.get_logger().info(
            f"{CYAN}[system_ready]{RESET} monitoring FCU / GPS / LiDAR / IMU ..."
        )

    def state_cb(self, msg):
        self.fcu_connected = msg.connected

    def fix_cb(self, msg):
        self.gps_fixed = msg.status.status >= 0

    def sat_cb(self, msg):
        self.satellites = msg.data

    def lidar_cb(self, msg):
        self.last_lidar = self.get_clock().now()

    def imu_cb(self, msg):
        self.last_imu = self.get_clock().now()

    def fresh(self, stamp):
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds / 1e9 <= self.sensor_timeout

    def check(self):
        lidar_ok = self.fresh(self.last_lidar)
        imu_ok = self.fresh(self.last_imu)

        if self.gps_fixed and not self.gps_announced:
            quality = "GOOD" if self.satellites >= self.min_satellites else "WEAK"
            colour = GREEN if self.satellites >= self.min_satellites else YELLOW
            self.get_logger().info(
                f"{colour}{BOLD}[GPS]{RESET} FIX ACQUIRED ({self.satellites} satellites, {quality})"
            )
            self.gps_announced = True
        if not self.gps_fixed and self.gps_announced:
            self.get_logger().warn(f"{YELLOW}[GPS]{RESET} FIX LOST")
            self.gps_announced = False

        ready = self.fcu_connected and self.gps_fixed and lidar_ok and imu_ok
        self.ready_pub.publish(Bool(data=ready))

        if ready and not self.all_ready:
            self.all_ready = True
            self.get_logger().info(
                f"{GREEN}{BOLD}"
                f"\n================================================\n"
                f"          ALL SYSTEMS READY\n"
                f"   FCU connected | GPS fix ({self.satellites} sats) | LiDAR OK | IMU OK\n"
                f"   -> safe to drive and start recording\n"
                f"================================================{RESET}"
            )
        elif not ready and self.all_ready:
            self.all_ready = False
            self.get_logger().warn(
                f"{YELLOW}[system_ready]{RESET} NO LONGER READY (a subsystem dropped)"
            )

        if not ready:
            self.throttled_status(self.fcu_connected, self.gps_fixed, lidar_ok, imu_ok)

    def throttled_status(self, fcu, gps, lidar, imu):
        now = self.get_clock().now()
        if (now - self.last_status_log).nanoseconds / 1e9 < 3.0:
            return
        self.last_status_log = now

        def mark(ok):
            return f"{GREEN}OK{RESET}" if ok else f"{RED}--{RESET}"

        self.get_logger().info(
            f"{CYAN}[waiting]{RESET} "
            f"FCU:{mark(fcu)} "
            f"GPS:{mark(gps)}({self.satellites} sats) "
            f"LiDAR:{mark(lidar)} "
            f"IMU:{mark(imu)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SystemReady()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
