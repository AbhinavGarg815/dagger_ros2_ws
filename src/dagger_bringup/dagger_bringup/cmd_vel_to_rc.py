#!/usr/bin/env python3
"""cmd_vel -> ArduPilot RC override (direct steering+throttle, MANUAL mode).

Why: this tracked rover has a steering+throttle mixer. In ArduPilot MANUAL,
RC steering(ch1)+throttle(ch3) drive the mixer directly and EVERYTHING works
(forward/back/arc/in-place pivot) — confirmed with the TX joystick. ArduPilot's
GUIDED velocity control, by contrast, treats it as a car and cannot synthesise a
valid turn command (it produced a +/-square-wave). So we drive the vehicle the
same way the TX does: convert cmd_vel to RC override PWM and let MANUAL + the
mixer do the rest.

  linear.x  -> throttle channel (RCMAP_THROTTLE=3), forward = >neutral
  angular.z -> steering channel (RCMAP_ROLL=1), sign set by steer_reversed
  in-place pivot = angular.z != 0 with linear.x = 0 (throttle neutral, mixer
                   counter-rotates the tracks)

Safety: streams overrides continuously (RC_OVERRIDE_TIME=10s on the FCU), and
publishes NEUTRAL on E-stop or command timeout (FS_THR_ENABLE=0, so the FCU
won't failsafe for us). Unused channels are sent 0 = "release to radio" so the
TX keeps mode control as a hardware override.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mavros_msgs.msg import OverrideRCIn

RELEASE = 0  # OverrideRCIn: 0 => release channel to the radio


class CmdVelToRC(Node):

    def __init__(self):
        super().__init__("cmd_vel_to_rc")

        self.declare_parameter("input_topic", "/cmd_vel_out")
        self.declare_parameter("estop_topic", "/estop")
        self.declare_parameter("steer_channel", 1)      # RCMAP_ROLL
        self.declare_parameter("throttle_channel", 3)   # RCMAP_THROTTLE
        self.declare_parameter("neutral_us", 1495)
        self.declare_parameter("min_us", 1045)
        self.declare_parameter("max_us", 1945)
        # cmd_vel magnitude that maps to full deflection
        self.declare_parameter("max_linear", 1.0)       # m/s -> full throttle
        self.declare_parameter("max_angular", 1.0)      # rad/s -> full steering
        self.declare_parameter("steer_reversed", True)  # Dagger: +angular.z -> left
        self.declare_parameter("throttle_reversed", False)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("command_timeout", 0.5)

        g = self.get_parameter
        self.input_topic = g("input_topic").value
        self.estop_topic = g("estop_topic").value
        self.steer_ch = int(g("steer_channel").value)
        self.thr_ch = int(g("throttle_channel").value)
        self.neutral = int(g("neutral_us").value)
        self.min_us = int(g("min_us").value)
        self.max_us = int(g("max_us").value)
        self.max_linear = float(g("max_linear").value)
        self.max_angular = float(g("max_angular").value)
        self.steer_sign = -1.0 if g("steer_reversed").value else 1.0
        self.thr_sign = -1.0 if g("throttle_reversed").value else 1.0
        self.publish_rate = float(g("publish_rate").value)
        self.command_timeout = float(g("command_timeout").value)

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.estop_active = False

        self.pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.create_subscription(Twist, self.input_topic, self.cmd_cb, 10)
        self.create_subscription(Bool, self.estop_topic, self.estop_cb, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_rc)

        self.get_logger().info(
            f"cmd_vel_to_rc: {self.input_topic} -> /mavros/rc/override "
            f"(steer ch{self.steer_ch}, throttle ch{self.thr_ch}, neutral "
            f"{self.neutral}us, max_lin={self.max_linear} max_ang={self.max_angular}, "
            f"steer_reversed={self.steer_sign < 0})"
        )

    def cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def estop_cb(self, msg):
        if msg.data and not self.estop_active:
            self.get_logger().warn("E-STOP engaged: RC neutral")
        if not msg.data and self.estop_active:
            self.get_logger().info("E-STOP cleared")
        self.estop_active = msg.data

    def _to_us(self, value, max_value, sign):
        """Map a cmd_vel component to a PWM microsecond value about neutral."""
        if max_value <= 0.0:
            frac = 0.0
        else:
            frac = max(-1.0, min(1.0, value / max_value))
        span = (self.max_us - self.min_us) / 2.0
        us = int(round(self.neutral + sign * frac * span))
        return max(self.min_us, min(self.max_us, us))

    def publish_rc(self):
        out = OverrideRCIn()
        out.channels = [RELEASE] * 18

        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if self.estop_active or elapsed > self.command_timeout:
            steer_us = self.neutral
            thr_us = self.neutral
        else:
            steer_us = self._to_us(self.last_cmd.angular.z, self.max_angular,
                                   self.steer_sign)
            thr_us = self._to_us(self.last_cmd.linear.x, self.max_linear,
                                 self.thr_sign)

        out.channels[self.steer_ch - 1] = steer_us
        out.channels[self.thr_ch - 1] = thr_us
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
