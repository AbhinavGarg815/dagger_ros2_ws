import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("dagger_bringup")
    kiss_config = os.path.join(pkg, "params", "kiss_icp.yaml")
    ekf_local_config = os.path.join(pkg, "params", "ekf_local.yaml")

    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    base_frame = LaunchConfiguration("base_frame")
    lidar_odom_frame = LaunchConfiguration("lidar_odom_frame")
    use_ekf = LaunchConfiguration("use_ekf")

    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic", default_value="/rslidar_points"
    )
    base_frame_arg = DeclareLaunchArgument("base_frame", default_value="base_footprint")
    lidar_odom_frame_arg = DeclareLaunchArgument("lidar_odom_frame", default_value="odom")
    use_ekf_arg = DeclareLaunchArgument("use_ekf", default_value="false")

    # KISS-ICP publishes odom->base_footprint TF only when the EKF is NOT running.
    # With use_ekf:=true the EKF owns odom->base_footprint and KISS only emits the
    # kiss/odometry topic that feeds the filter (avoids two publishers fighting for
    # the same transform).
    kiss_publish_odom_tf = ParameterValue(
        PythonExpression(["'", use_ekf, "' == 'false'"]), value_type=bool
    )

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
                "publish_odom_tf": kiss_publish_odom_tf,
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
        use_ekf_arg,
        kiss_icp_node,
        ekf_local_node,
    ])
