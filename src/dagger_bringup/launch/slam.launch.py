"""2D SLAM (slam_toolbox) -- T5, independent of Nav2 and follow-me.

    ros2 launch dagger_bringup slam.launch.py                  # SHADOW mapping (default)
    ros2 launch dagger_bringup slam.launch.py mode:=mapping    # same

SHADOW (default, publish_map_tf:=false): builds a map from /scan + our fused odom
but publishes NOTHING on /tf -- map frame renamed slam_map, outputs under /slam/.
Nav2, the GPS map layer (gps_map_odom owns map->odom) and follow-me are
untouched, so it can run alongside any test to judge map quality and CPU.
Save a map: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap
            "{name: {data: '/home/jetson/ros2_ws/src/dagger_bringup/maps/<name>'}}"

Handing slam the real map frame (publish_map_tf:=true) is a later step and needs
gps_map_odom switched off first -- odom can have only one parent in TF.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("dagger_bringup")
    publish_tf = LaunchConfiguration("publish_map_tf")
    publish_tf_arg = DeclareLaunchArgument("publish_map_tf", default_value="false")

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(bringup, "params", "slam_toolbox.yaml"),
            {
                "mode": "mapping",
                # shadow: own frame name and no TF at all
                "map_frame": PythonExpression(["'map' if '", publish_tf, "' == 'true' else 'slam_map'"]),
                "transform_publish_period": PythonExpression(
                    ["0.05 if '", publish_tf, "' == 'true' else 0.0"]),
            },
        ],
        remappings=[("map", "/slam/map"), ("map_metadata", "/slam/map_metadata"),
                    ("pose", "/slam/pose")],
    )
    return LaunchDescription([publish_tf_arg, slam])
