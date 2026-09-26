import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    # Stable by-id path (if00 = Cube MAVLink port) — survives USB re-enumeration.
    # ttyACM numbering shifts when GNSS/radio hardware changes; by-id does not.
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/serial/by-id/usb-CubePilot_CubeOrange+_44002F000E51333233343437-if00:57600",
    )

    # D435 colour stream for follow-me. Off by default: it is not needed for
    # driving, and the detector is the only consumer. sensors.launch.py has its
    # own use_camera flag, but T1 is this file and never included it, so the flag
    # was unreachable from the real bring-up.
    use_camera = LaunchConfiguration("use_camera")
    use_camera_arg = DeclareLaunchArgument("use_camera", default_value="false")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dagger_description"), "launch", "rsp.launch.py"
            )
        )
    )

    lidar = Node(
        namespace="rslidar_sdk",
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        output="screen",
    )

    # mavros loads all 65 plugins by default and used a full core (102 %) on
    # 2026-09-26, with the Orin saturated at ~495 % of 600 % and the follow
    # collision monitor starving as a result. The stack uses 13. In mavros a
    # plugin is disabled when it matches the denylist AND not the allowlist, so
    # the pattern is deny "*", allow ours. mavros_plugins:=all restores the lot.
    #   sys_status  /mavros/state, set_mode, arming     command  cmd/command
    #   sys_time    time sync                           param    FCU params
    #   rc_io       rc/out, rc/in                       imu      imu/data(_raw)
    #   local_position, global_position, home_position, gps_status (gps1/raw)
    #   setpoint_velocity  the GUIDED command path
    #   setpoint_raw       FCU echo of the executed target (latency checks)
    #   waypoint           /mavros/mission (tools/gps_goal.py --mission)
    # Not included, add when needed: gps_rtk (RTCM injection over the RFD868),
    # vision_pose (the superseded ExtNav yaw relay).
    mavros_plugins = LaunchConfiguration("mavros_plugins")
    mavros_plugins_arg = DeclareLaunchArgument("mavros_plugins", default_value="minimal")
    minimal = ["sys_status", "sys_time", "command", "param", "rc_io", "imu",
               "local_position", "global_position", "home_position", "gps_status",
               "setpoint_velocity", "setpoint_raw", "waypoint"]
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", mavros_plugins, "' == 'minimal'"])),
        parameters=[{"fcu_url": fcu_url, "plugin_denylist": ["*"], "plugin_allowlist": minimal}],
    )
    mavros_node_all = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", mavros_plugins, "' == 'all'"])),
        parameters=[{"fcu_url": fcu_url}],
    )

    system_ready = Node(
        package="dagger_bringup",
        executable="system_ready",
        name="system_ready",
        output="screen",
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dagger_bringup"), "launch", "camera.launch.py"
            )
        ),
        condition=IfCondition(use_camera),
    )

    return LaunchDescription(
        [fcu_url_arg, use_camera_arg, mavros_plugins_arg, rsp, lidar, mavros_node,
         mavros_node_all, system_ready, camera]
    )
