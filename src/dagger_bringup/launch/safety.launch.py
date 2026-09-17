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

    output_topic = LaunchConfiguration("output_topic")
    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/mavros/setpoint_velocity/cmd_vel_unstamped",
    )
    # Sensor freshness gate for cmd_vel_safety (zero output while this LaserScan is
    # stale). nav2.launch.py sets /scan_safety; empty = off (e.g. teleop-only use).
    fresh_topic = LaunchConfiguration("fresh_topic")
    fresh_topic_arg = DeclareLaunchArgument("fresh_topic", default_value="")

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[params],
        remappings=[("cmd_vel_out", "/cmd_vel_out")],
    )

    cmd_vel_safety = Node(
        package="dagger_bringup",
        executable="cmd_vel_safety",
        name="cmd_vel_safety",
        output="screen",
        parameters=[{
            "input_topic": "/cmd_vel_out",
            "output_topic": output_topic,
            "estop_topic": "/estop",
            "publish_rate": 20.0,
            "command_timeout": 0.5,
            "fresh_topic": fresh_topic,
            "fresh_timeout": 0.5,
        }],
    )

    return LaunchDescription([output_topic_arg, fresh_topic_arg, twist_mux, cmd_vel_safety])
