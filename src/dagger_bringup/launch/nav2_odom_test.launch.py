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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("dagger_bringup")

    # ExtNav yaw relay ON by default here: feed our clean EKF yaw to ArduPilot so
    # GUIDED turns work compassless (needs FCU EK3_SRC1_YAW=6). Auto earth-aligns
    # from GPS course during the first straight drive; override with a fixed offset
    # via extnav_manual_offset_deg:=<deg>.
    extnav_yaw = LaunchConfiguration("extnav_yaw")
    extnav_manual_offset_deg = LaunchConfiguration("extnav_manual_offset_deg")
    # OFF by default: dual-antenna C-RTK 2HP now gives true GPS yaw (EK3_SRC1_YAW=2),
    # so the ExtNav relay is superseded. Kept as an opt-in fallback (extnav_yaw:=true).
    extnav_yaw_arg = DeclareLaunchArgument("extnav_yaw", default_value="false")
    extnav_manual_offset_arg = DeclareLaunchArgument(
        "extnav_manual_offset_deg", default_value="nan"
    )
    # fcu | fused | kiss -- see localization.launch.py. DEFAULT fused = EK3 velocity +
    # raw gyro EKF (no GPS position in odom); validated 2026-09-14 (legs A/C).
    odom_source = LaunchConfiguration("odom_source")
    odom_source_arg = DeclareLaunchArgument("odom_source", default_value="fused")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "use_ekf": "true",
            "odom_source": odom_source,
            "extnav_yaw": extnav_yaw,
            "extnav_manual_offset_deg": extnav_manual_offset_deg,
        }.items(),
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

    # Identity map->odom PLACEHOLDER (2026-09-14). Nav2 here plans in odom, but some
    # BT nodes (RemovePassedGoals in NavigateThroughPoses) look the robot up in
    # 'map' by default -> TF error -> goal abort. This stands in for the future
    # GPS map->odom layer, which will replace it (same REP-105 frame tree).
    map_odom_placeholder = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_placeholder",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        output="screen",
    )

    return LaunchDescription([
        map_odom_placeholder,
        extnav_yaw_arg,
        extnav_manual_offset_arg,
        odom_source_arg,
        localization,
        nav2,
    ])
