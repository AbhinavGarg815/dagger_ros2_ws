import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg = get_package_share_directory("dagger_description")
    xacro_file = os.path.join(pkg, "urdf", "robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    # Only the two wheel joints move, and nothing but the Foxglove robot model uses
    # their frames (all sensors hang off fixed joints): 2 Hz instead of the default
    # 10 Hz (~6 % CPU, 2026-09-29).
    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"rate": 2}],
    )

    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_link_to_rslidar",
        arguments=["0", "0", "0", "0", "0", "0", "lidar_link", "rslidar"],
        output="screen",
    )

    return LaunchDescription([rsp, jsp, lidar_static_tf])
