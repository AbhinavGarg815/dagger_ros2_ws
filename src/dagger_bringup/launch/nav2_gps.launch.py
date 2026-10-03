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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("dagger_bringup")

    tau_xy = LaunchConfiguration("tau_xy")
    tau_yaw = LaunchConfiguration("tau_yaw")
    tau_xy_arg = DeclareLaunchArgument("tau_xy", default_value="5.0")
    tau_yaw_arg = DeclareLaunchArgument("tau_yaw", default_value="10.0")
    # Who owns map->odom: gps (gps_map_odom, map = ArduPilot local frame) or
    # slam (T5 slam.launch.py mode:=localization publishes it; map = the SLAM map).
    # Never both -- odom can have only one parent.
    map_source = LaunchConfiguration("map_source")
    map_source_arg = DeclareLaunchArgument("map_source", default_value="gps",
                                           choices=["gps", "slam"])
    # Source of /scan_safety (both collision monitors): see nav2.launch.py.
    near_field = LaunchConfiguration("near_field")
    near_field_arg = DeclareLaunchArgument("near_field", default_value="false",
                                           choices=["true", "false"])
    # Low-obstacle memory (low_scan + the costmaps' low_obstacle_layer): see nav2.launch.py.
    low_memory = LaunchConfiguration("low_obstacle_memory")
    low_memory_arg = DeclareLaunchArgument("low_obstacle_memory", default_value="false",
                                           choices=["true", "false"])

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
        condition=IfCondition(PythonExpression(["'", map_source, "' == 'gps'"])),
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
            "near_field": near_field,
            "low_obstacle_memory": low_memory,
        }.items(),
    )

    return LaunchDescription([tau_xy_arg, tau_yaw_arg, map_source_arg, near_field_arg, low_memory_arg,
                              localization, gps_map_odom, nav2])
