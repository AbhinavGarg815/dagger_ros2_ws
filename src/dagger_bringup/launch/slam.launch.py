"""2D SLAM (slam_toolbox) -- T5.

    ros2 launch dagger_bringup slam.launch.py                              # SHADOW mapping (default)
    ros2 launch dagger_bringup slam.launch.py mode:=localization map:=lane3fix
    ros2 launch dagger_bringup slam.launch.py mode:=localization shadow:=true map:=/abs/Yard

SHADOW localization (shadow:=true, the app's map missions, map_manager): localizes
on the saved map in its own slam_map frame and publishes NOTHING into the TF tree,
like shadow mapping -- gps_map_odom keeps map->odom, Nav2 is untouched. The
gateway reads /slam/pose and /slam/localized and sends a map mission's goals
relative to the rover's SLAM pose, in odom.

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
Foxglove's pose-estimate tool can override it. Until then SLAM sits at the map
origin (map_start_pose 0,0,0 -- needed, or the map is not loaded at all). /slam/localized says when the scan fits the map.
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
    shadow = context.launch_configurations["shadow"] == "true"
    frame = "slam_map" if shadow else "map"
    name = context.launch_configurations["map"]
    maps = os.path.join(bringup, "maps")
    base = name if os.path.isabs(name) else os.path.join(maps, name)
    # A map not tied to the Earth (Step D2, made indoors) has no _geo.yaml: shadow
    # localization still runs on it, started from "I am here" instead of GPS.
    needed = (".posegraph", ".data", ".yaml") + (() if shadow else ("_geo.yaml",))
    geo = base + "_geo.yaml"
    for ext in needed:
        if not os.path.exists(base + ext):
            raise RuntimeError(f"missing {base + ext} (build it with tools/slam_replay.sh "
                               "and tools/slam_georef.py)")
    return [
        Node(package="slam_toolbox", executable="localization_slam_toolbox_node",
             name="slam_toolbox", output="screen", remappings=remaps,
             parameters=[params, {"mode": "localization", "map_frame": frame,
                                  "transform_publish_period": 0.0 if shadow else 0.05,
                                  "map_file_name": base,
                                  # REQUIRED: without a start pose the localization node
                                  # silently does NOT load the map ("Map starting pose not
                                  # specified") and just tracks scan-to-scan from the seed
                                  # (0b70017, loc1 drive invalid). The real start pose comes
                                  # from slam_geo_init on /initialpose.
                                  "map_start_pose": [0.0, 0.0, 0.0]}]),
        Node(package="dagger_bringup", executable="slam_geo_init",
             name="slam_geo_init", output="screen",
             parameters=[{"geo_file": geo if os.path.exists(geo) else "", "map_yaml": base + ".yaml",
                          "map_frame": frame}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mapping",
                              description="mapping (shadow) | localization"),
        DeclareLaunchArgument("shadow", default_value="false", choices=["true", "false"],
                              description="localization: true = own slam_map frame, no TF (map missions)"),
        DeclareLaunchArgument("map", default_value="lane3fix",
                              description="localization: map name in maps/ or absolute path, no extension"),
        OpaqueFunction(function=setup),
    ])
