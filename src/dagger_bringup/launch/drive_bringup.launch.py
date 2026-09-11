import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    # Stable by-id path (if00 = Cube MAVLink port) — survives USB re-enumeration.
    # ttyACM numbering shifts when GNSS/radio hardware changes; by-id does not.
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/serial/by-id/usb-CubePilot_CubeOrange+_44002F000E51333233343437-if00:57600",
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dagger_description"), "launch", "rsp.launch.py"
            )
        )
    )

    lidar = Node(
        namespace="rslidar_sdk",
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        output="screen",
    )

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[{"fcu_url": fcu_url}],
    )

    system_ready = Node(
        package="dagger_bringup",
        executable="system_ready",
        name="system_ready",
        output="screen",
    )

    return LaunchDescription(
        [fcu_url_arg, rsp, lidar, mavros_node, system_ready]
    )
