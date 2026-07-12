import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("dagger_bringup")

    params_file = LaunchConfiguration("params_file")
    extra_params = LaunchConfiguration("extra_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    default_params = os.path.join(bringup_dir, "params", "nav2_dagger.yaml")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
    )
    # Optional overlay applied AFTER params_file (later file wins per-parameter).
    # Defaults to params_file itself (a harmless no-op re-load); the odom-only
    # validation passes nav2_odom_test.yaml here to override global_frame.
    extra_params_arg = DeclareLaunchArgument(
        "extra_params",
        default_value=default_params,
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")

    nav2_params = [params_file, extra_params]

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", "/rslidar_points"),
            ("scan", "/scan"),
        ],
        parameters=[{
            "target_frame": "base_footprint",
            "transform_tolerance": 0.1,
            "min_height": 0.15,
            "max_height": 0.90,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.5,
            "range_max": 30.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
            "use_sim_time": use_sim_time,
        }],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=nav2_params,
        remappings=[("cmd_vel", "/cmd_vel_nav")],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=nav2_params,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=nav2_params,
        remappings=[("cmd_vel", "/cmd_vel_nav")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=nav2_params,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
            ],
        }],
    )

    # Direct RC-override command path (MANUAL mode). Replaces the old GUIDED
    # velocity path (safety.launch.py) because ArduPilot GUIDED can't turn this
    # tracked/mixer vehicle; direct steering+throttle via RC override does.
    rc_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rc_control.launch.py")
        )
    )

    return LaunchDescription([
        params_file_arg,
        extra_params_arg,
        use_sim_time_arg,
        autostart_arg,
        pointcloud_to_laserscan,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
        rc_control,
    ])
