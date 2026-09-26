"""2D SLAM (slam_toolbox) -- T5.

    ros2 launch dagger_bringup slam.launch.py                              # SHADOW mapping (default)
    ros2 launch dagger_bringup slam.launch.py mode:=localization map:=lane3fix

SHADOW mapping (default): builds a map from /scan + our fused odom but publishes
NOTHING into the real TF tree -- map frame renamed slam_map, outputs under
/slam/. Nav2, the GPS map layer (gps_map_odom owns map->odom) and follow-me are
untouched, so it can run alongside any test.
Save a map:  ros2 run nav2_map_server map_saver_cli -t /slam/map -f <path> \
               --ros-args -p map_subscribe_transient_local:=true
             ros2 service call /slam_toolbox/serialize_map \
               slam_toolbox/srv/SerializePoseGraph "{filename: '<path>'}"

LOCALIZATION: localizes on the saved pose graph maps/<map>.posegraph and OWNS
map->odom. T2 must then run with map_source:=slam (no gps_map_odom) -- odom can
have only one parent. The start pose: GNSS (through maps/<map>_geo.yaml) narrows a scan-to-map
search on maps/<map>.pgm, and the winner goes to /initialpose (slam_geo_init);
Foxglove's pose-estimate tool can override it. Until then the map is loaded at
the mapping start point. /slam/localized says when the scan fits the map.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def setup(context):
    bringup = get_package_share_directory("dagger_bringup")
    mode = context.launch_configurations["mode"]
    params = os.path.join(bringup, "params", "slam_toolbox.yaml")
    remaps = [("map", "/slam/map"), ("map_metadata", "/slam/map_metadata"),
              ("pose", "/slam/pose")]

    if mode == "mapping":
        return [
            Node(package="slam_toolbox", executable="async_slam_toolbox_node",
                 name="slam_toolbox", output="screen", remappings=remaps,
                 parameters=[params, {"mode": "mapping", "map_frame": "slam_map",
                                      "transform_publish_period": 0.0}]),
            # slam_map as the root of its own tiny TF tree (never touching odom),
            # so Foxglove can use it as a display frame.
            Node(package="dagger_bringup", executable="slam_pose_tf",
                 name="slam_pose_tf", output="screen"),
        ]

    if mode != "localization":
        raise RuntimeError(f"mode must be mapping or localization, not '{mode}'")
    name = context.launch_configurations["map"]
    maps = os.path.join(bringup, "maps")
    base = name if os.path.isabs(name) else os.path.join(maps, name)
    for ext in (".posegraph", ".data", ".yaml", "_geo.yaml"):
        if not os.path.exists(base + ext):
            raise RuntimeError(f"missing {base + ext} (build it with tools/slam_replay.sh "
                               "and tools/slam_georef.py)")
    return [
        Node(package="slam_toolbox", executable="localization_slam_toolbox_node",
             name="slam_toolbox", output="screen", remappings=remaps,
             parameters=[params, {"mode": "localization", "map_frame": "map",
                                  "transform_publish_period": 0.05,
                                  "map_file_name": base,
                                  "map_start_at_dock": True}]),
        Node(package="dagger_bringup", executable="slam_geo_init",
             name="slam_geo_init", output="screen",
             parameters=[{"geo_file": base + "_geo.yaml", "map_yaml": base + ".yaml"}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mapping",
                              description="mapping (shadow) | localization"),
        DeclareLaunchArgument("map", default_value="lane3fix",
                              description="localization: map name in maps/ or absolute path, no extension"),
        OpaqueFunction(function=setup),
    ])
