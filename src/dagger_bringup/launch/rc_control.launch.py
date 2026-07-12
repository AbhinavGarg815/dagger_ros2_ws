"""Direct-control command path: twist_mux -> cmd_vel_to_rc -> RC override (MANUAL).

Replaces the GUIDED velocity path (safety.launch.py's cmd_vel_safety) with direct
RC override, because ArduPilot GUIDED can't turn this tracked/mixer vehicle but
MANUAL + direct steering/throttle works perfectly (proven with the TX joystick).

Run ArduPilot in MANUAL + armed. twist_mux still arbitrates teleop > nav > follow
and the E-stop lock, and cmd_vel_to_rc carries the E-stop + watchdog to RC neutral.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("dagger_bringup"), "params", "twist_mux.yaml"
    )

    max_linear = LaunchConfiguration("max_linear")
    max_angular = LaunchConfiguration("max_angular")
    steer_reversed = LaunchConfiguration("steer_reversed")
    throttle_reversed = LaunchConfiguration("throttle_reversed")

    args = [
        DeclareLaunchArgument("max_linear", default_value="1.0",
                              description="cmd_vel linear.x (m/s) mapped to full throttle"),
        DeclareLaunchArgument("max_angular", default_value="1.0",
                              description="cmd_vel angular.z (rad/s) mapped to full steering"),
        DeclareLaunchArgument("steer_reversed", default_value="true",
                              description="Dagger's steering is inverted vs cmd_vel; "
                                          "true makes +angular.z pivot left"),
        DeclareLaunchArgument("throttle_reversed", default_value="false"),
    ]

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[params],
        remappings=[("cmd_vel_out", "/cmd_vel_out")],
    )

    cmd_vel_to_rc = Node(
        package="dagger_bringup",
        executable="cmd_vel_to_rc",
        name="cmd_vel_to_rc",
        output="screen",
        parameters=[{
            "input_topic": "/cmd_vel_out",
            "estop_topic": "/estop",
            "steer_channel": 1,
            "throttle_channel": 3,
            "neutral_us": 1495,
            "min_us": 1045,
            "max_us": 1945,
            "max_linear": max_linear,
            "max_angular": max_angular,
            "steer_reversed": steer_reversed,
            "throttle_reversed": throttle_reversed,
            "publish_rate": 30.0,
            "command_timeout": 0.5,
        }],
    )

    return LaunchDescription([*args, twist_mux, cmd_vel_to_rc])
