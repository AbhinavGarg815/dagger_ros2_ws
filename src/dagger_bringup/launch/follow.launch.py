"""Follow-me (object following) stack -- T4, started and stopped independently of Nav2.

    T1 drive_bringup -> T2 nav2_gps -> T3 foxglove -> T4 follow

Deliberately NOT folded into nav2.launch.py: follow-me must be startable and
killable without touching Nav2, and a follow node that fails to activate must not
stop Nav2 from coming up (a shared lifecycle manager would do exactly that).

It does DEPEND on T2 being up, for three things it does not own:
  * /scan_safety            (p2l_safety)        -- the monitor's obstacle source
  * twist_mux + cmd_vel_safety (safety.launch)  -- the path to the FCU
  * /local_costmap/published_footprint          -- the approach polygon's shape
See collision_monitor_follow.yaml for why the third one fails SILENTLY if absent.

Stage 1 contains the safety gate only. The detector, target selector and follow
controller land here later, publishing /cmd_vel_follow_raw into this monitor.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("dagger_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")

    # /cmd_vel_follow_raw -> stop / slowdown / approach -> /cmd_vel_follow.
    # Same zones and same /scan_safety as the Nav2 monitor, so the rover stops at
    # the same distance from a person whichever path is driving.
    collision_monitor_follow = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor_follow",
        output="screen",
        parameters=[
            os.path.join(bringup_dir, "params", "collision_monitor_follow.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # Own manager: follow-me's lifecycle is independent of Nav2's.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_follow",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": ["collision_monitor_follow"],
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        collision_monitor_follow,
        lifecycle_manager,
    ])
