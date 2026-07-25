from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mavros_fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/ttyACM0:57600"
    )
    
    fcu_url = LaunchConfiguration("fcu_url")
    
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        parameters=[{
            "fcu_url": fcu_url
        }]
    )
    
    lidar = Node(
        namespace="rslidar_sdk",
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        output="screen",
    )
    
    pkg_path = get_package_share_directory("dagger_description")
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, "launch", "rsp.launch.py")
        )
    )

    return LaunchDescription([
        mavros_fcu_url_arg,
        lidar,
        rsp,
        mavros_node,
    ])
