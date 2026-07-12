"""Isolated odom-only turn-fix validation (Step 2 of the odom-decouple).

Brings up the decoupled fast-odom source (KISS-ICP -> EKF, publishing
odom->base_footprint at 30 Hz) and Nav2 running purely in the `odom` frame,
WITHOUT mrg_slam. This isolates whether the fast EKF odom fixes the 180-deg
in-place turn, independent of map->odom localization.

Prerequisites (run first, in another terminal):
    drive_bringup.launch.py   # mavros (IMU), lidar, robot_state_publisher

Then:
    ros2 launch dagger_bringup nav2_odom_test.launch.py

mrg_slam / slam3d must NOT be running (it would fight for odom->base).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory("dagger_bringup")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "localization.launch.py")
        ),
        launch_arguments={"use_ekf": "true"}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "extra_params": os.path.join(
                bringup_dir, "params", "nav2_odom_test.yaml"
            ),
        }.items(),
    )

    return LaunchDescription([localization, nav2])
