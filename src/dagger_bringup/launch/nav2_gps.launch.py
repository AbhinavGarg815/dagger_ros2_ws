"""GPS navigation: fused odom + GPS map->odom layer + Nav2 planning in `map`.

Frames: map (= ArduPilot EK3 local ENU frame) -> odom (fused EK3 velocity + raw
gyro, never jumps) -> base_footprint. gps_map_odom low-pass-drags map->odom onto
the EK3 GPS pose, so GPS/lat-lon goals (tools/gps_goal.py) land in the same frame
Mission Planner AUTO navigates in. Costmaps + planning run in odom (see below).

Prerequisite (other terminal): drive_bringup.launch.py, then tools/preflight.py.
    ros2 launch dagger_bringup nav2_gps.launch.py
Replaces nav2_odom_test.launch.py (which publishes an identity map->odom
placeholder) -- do not run both.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("dagger_bringup")

    tau_xy = LaunchConfiguration("tau_xy")
    tau_yaw = LaunchConfiguration("tau_yaw")
    tau_xy_arg = DeclareLaunchArgument("tau_xy", default_value="5.0")
    tau_yaw_arg = DeclareLaunchArgument("tau_yaw", default_value="10.0")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "localization.launch.py")),
        launch_arguments={"use_ekf": "true", "odom_source": "fused"}.items(),
    )

    gps_map_odom = Node(
        package="dagger_bringup",
        executable="gps_map_odom",
        name="gps_map_odom",
        output="screen",
        parameters=[{
            "tau_xy": ParameterValue(tau_xy, value_type=float),
            "tau_yaw": ParameterValue(tau_yaw, value_type=float),
        }],
    )

    # Costmaps + planning stay in ODOM (nav2_odom_test.yaml overlay). A global
    # costmap in `map` stores obstacles in the GPS frame: when map->odom slid 8.5 m
    # in 80 s (non-RTK walk + pivot transients, 2026-09-14) remembered walls slid
    # with it -> ghost obstacles -> the path flipped every replan -> in-place hunt.
    # GPS now only decides where lat/lon goals land (goals held in `map`, re-read
    # with the latest map->odom each replan).
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "nav2.launch.py")),
        launch_arguments={
            "extra_params": os.path.join(bringup_dir, "params", "nav2_odom_test.yaml"),
        }.items(),
    )

    return LaunchDescription([tau_xy_arg, tau_yaw_arg, localization, gps_map_odom, nav2])
