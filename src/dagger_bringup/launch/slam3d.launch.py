import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mrg_launch = os.path.join(
        get_package_share_directory("mrg_slam"), "launch", "mrg_slam.launch.py"
    )

    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_arg = DeclareLaunchArgument("config", default_value="dagger.yaml")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mrg_launch),
        launch_arguments={
            "config": config,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    base_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="dagger_base_to_footprint",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "dagger/base_link",
            "--child-frame-id", "base_footprint",
        ],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        output="screen",
    )

    gps_relay = Node(
        package="dagger_bringup",
        executable="gps_relay",
        name="gps_relay",
        output="screen",
        parameters=[{
            "input_topic": "/mavros/global_position/raw/fix",
            "output_topic": "/dagger/gps/navsat",
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    return LaunchDescription([config_arg, use_sim_time_arg, slam, base_bridge, gps_relay])
