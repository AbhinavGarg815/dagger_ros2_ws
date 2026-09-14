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
    ekf_fused_config = os.path.join(pkg, "params", "ekf_fused.yaml")

    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    base_frame = LaunchConfiguration("base_frame")
    lidar_odom_frame = LaunchConfiguration("lidar_odom_frame")
    use_ekf = LaunchConfiguration("use_ekf")
    odom_source = LaunchConfiguration("odom_source")
    extnav_yaw = LaunchConfiguration("extnav_yaw")
    extnav_manual_offset_deg = LaunchConfiguration("extnav_manual_offset_deg")

    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic", default_value="/rslidar_points"
    )
    base_frame_arg = DeclareLaunchArgument("base_frame", default_value="base_footprint")
    lidar_odom_frame_arg = DeclareLaunchArgument("lidar_odom_frame", default_value="odom")
    use_ekf_arg = DeclareLaunchArgument("use_ekf", default_value="false")
    # odom_source: 'fused' (DEFAULT since 2026-09-14) | 'fcu' = ArduPilot EK3 pose via
    # fcu_odom_relay (GPS-anchored: walks 1-3 m/min without RTK -> not for Nav2).
    # 'kiss' = legacy KISS-ICP + robot_localization EKF (jumps during rotation).
    # 'fused' = robot_localization EKF on EK3 VELOCITY + raw gyro only (no GPS
    # position/yaw) -> continuous odom that never jumps (params/ekf_fused.yaml).
    odom_source_arg = DeclareLaunchArgument("odom_source", default_value="fused")
    use_kiss = PythonExpression(["'", odom_source, "' == 'kiss'"])
    use_fcu = PythonExpression(["'", odom_source, "' == 'fcu'"])
    use_fused = PythonExpression(["'", odom_source, "' == 'fused'"])
    # ExtNav yaw relay: feed our drift-free EKF yaw to ArduPilot as VISION yaw so
    # GUIDED works compassless (needs FCU EK3_SRC1_YAW=6). Off by default.
    extnav_yaw_arg = DeclareLaunchArgument("extnav_yaw", default_value="false")
    extnav_manual_offset_arg = DeclareLaunchArgument(
        "extnav_manual_offset_deg", default_value="nan",
        description="Force earth-align offset (deg); 'nan' = auto-align from GPS course",
    )

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
        condition=IfCondition(use_kiss),
    )

    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[ekf_local_config],
        condition=IfCondition(PythonExpression(
            ["'", odom_source, "' == 'kiss' and '", use_ekf, "' == 'true'"])),
    )

    # ArduPilot EK3 pose -> /odometry/filtered + odom->base_footprint TF (GPS-based,
    # rotation-robust). Used when odom_source:=fcu (legacy; walks without RTK).
    fcu_odom_relay_node = Node(
        package="dagger_bringup",
        executable="fcu_odom_relay",
        name="fcu_odom_relay",
        output="screen",
        parameters=[{
            "input_topic": "/mavros/local_position/odom",
            "output_topic": "/odometry/filtered",
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "flatten_z": True,
            "publish_tf": True,
        }],
        condition=IfCondition(use_fcu),
    )

    # odom_source:=fused -- the relay only re-labels EK3 odom (+ realistic twist
    # covariance) onto /fcu/odom, NO TF; ekf_local fuses its velocity + raw gyro
    # and alone owns /odometry/filtered + odom->base_footprint.
    fcu_twist_relay_node = Node(
        package="dagger_bringup",
        executable="fcu_odom_relay",
        name="fcu_odom_relay",
        output="screen",
        parameters=[{
            "input_topic": "/mavros/local_position/odom",
            "output_topic": "/fcu/odom",
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "flatten_z": True,
            "publish_tf": False,
            "twist_variance": [0.01, 0.01, 0.01],
            "zero_velocity_deadband": 0.02,
        }],
        condition=IfCondition(use_fused),
    )

    # Dual-antenna GNSS heading -> Imu(yaw) for ekf_local (fused mode only).
    gps_heading_node = Node(
        package="dagger_bringup",
        executable="gps_heading_imu",
        name="gps_heading_imu",
        output="screen",
        condition=IfCondition(use_fused),
    )

    ekf_fused_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[ekf_fused_config],
        remappings=[("odometry/filtered", "/odometry/filtered")],
        condition=IfCondition(use_fused),
    )

    extnav_yaw_relay = Node(
        package="dagger_bringup",
        executable="extnav_yaw_relay",
        name="extnav_yaw_relay",
        output="screen",
        parameters=[{
            "odom_topic": "/odometry/filtered",
            "gps_vel_topic": "/mavros/global_position/raw/gps_vel",
            "output_topic": "/mavros/vision_pose/pose",
            "manual_offset_deg": ParameterValue(
                extnav_manual_offset_deg, value_type=float),
            "auto_align": True,
        }],
        condition=IfCondition(extnav_yaw),
    )

    return LaunchDescription([
        pointcloud_topic_arg,
        base_frame_arg,
        lidar_odom_frame_arg,
        use_ekf_arg,
        odom_source_arg,
        extnav_yaw_arg,
        extnav_manual_offset_arg,
        kiss_icp_node,
        ekf_local_node,
        fcu_odom_relay_node,
        fcu_twist_relay_node,
        gps_heading_node,
        ekf_fused_node,
        extnav_yaw_relay,
    ])
