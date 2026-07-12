import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("dagger_bringup")
    kiss_config = os.path.join(pkg, "params", "kiss_icp.yaml")
    ekf_local_config = os.path.join(pkg, "params", "ekf_local.yaml")

    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    base_frame = LaunchConfiguration("base_frame")
    lidar_odom_frame = LaunchConfiguration("lidar_odom_frame")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    use_ekf = LaunchConfiguration("use_ekf")

    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic", default_value="/rslidar_points"
    )
    base_frame_arg = DeclareLaunchArgument("base_frame", default_value="base_footprint")
    lidar_odom_frame_arg = DeclareLaunchArgument("lidar_odom_frame", default_value="odom")
    publish_odom_tf_arg = DeclareLaunchArgument("publish_odom_tf", default_value="true")
    use_ekf_arg = DeclareLaunchArgument("use_ekf", default_value="false")

    kiss_icp_node = Node(
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        remappings=[("pointcloud_topic", pointcloud_topic)],
        parameters=[
            kiss_config,
            {
                "base_frame": base_frame,
                "lidar_odom_frame": lidar_odom_frame,
                "publish_odom_tf": publish_odom_tf,
                "invert_odom_tf": True,
                "publish_debug_clouds": False,
                "use_sim_time": False,
                "position_covariance": 0.1,
                "orientation_covariance": 0.1,
            },
        ],
    )

    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[ekf_local_config],
        condition=IfCondition(use_ekf),
    )

    return LaunchDescription([
        pointcloud_topic_arg,
        base_frame_arg,
        lidar_odom_frame_arg,
        publish_odom_tf_arg,
        use_ekf_arg,
        kiss_icp_node,
        ekf_local_node,
    ])
