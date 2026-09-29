"""Foxglove bridge for Dagger, locked down for a 350 kg vehicle.

    ros2 launch dagger_bringup foxglove.launch.py              # no raw point cloud
    ros2 launch dagger_bringup foxglove.launch.py with_cloud:=true

Then in Foxglove: Open connection -> Foxglove WebSocket -> ws://<jetson-wifi-ip>:8765

Safety / load choices (2026-09-17):
- Clients may PUBLISH only /goal_pose_ui and /initialpose_ui (the 3D panel's
  publish tool: "Pose" and "Pose estimate" -- set those topics in the panel),
  and /estop (std_msgs/Bool; one `true` latches the twist_mux lock and
  cmd_vel_safety until `false`). No cmd_vel, no arming. goal_pose_relay forwards
  the two *_ui topics (2026-09-29): Foxglove stamps the click time, and Nav2
  re-transforms goals at that stamp every replan -> TF ages out after ~10 s ->
  abort; the relay sends stamp 0, converts moving frames into map, refuses goals
  while SLAM is not localized. /goal_pose itself is no longer client-publishable.
- No /mavros services or parameters exposed: nothing in the UI can arm, change
  mode, or write FCU params. Nav2 params stay editable (live tuning).
- /rslidar_points (~0.8 MB x 10 Hz) is hidden by default: streaming it over Wi-Fi
  cost ~1 Orin core and starved the controller in July. Use /scan + costmaps.
- asset_uri_allowlist extended to upper-case .STL: dagger_description meshes are
  named *.STL and the stock filter only matches lower-case, so the robot model
  would silently fail to load.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bridge(context):
    with_cloud = LaunchConfiguration("with_cloud").perform(context).lower() == "true"
    port = int(LaunchConfiguration("port").perform(context))
    topics = [".*"] if with_cloud else ["^(?!/rslidar_points$).*"]
    return [Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": port,
            "address": "0.0.0.0",
            "topic_whitelist": topics,
            "client_topic_whitelist": ["^/goal_pose_ui$", "^/initialpose_ui$", "^/estop$"],
            "service_whitelist": ["^(?!/mavros).*"],
            "param_whitelist": ["^(?!/mavros).*"],
            "send_buffer_limit": 10000000,
            "asset_uri_allowlist": [
                r"^package://(?:[-\w%]+/)*[-\w%.]+\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png"
                r"|stl|STL|tif|tiff|urdf|webp|xacro)$"],
        }],
    ), Node(
        package="dagger_bringup",
        executable="goal_pose_relay",
        name="goal_pose_relay",
        output="screen",
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("with_cloud", default_value="false",
                              description="also stream the raw /rslidar_points cloud"),
        DeclareLaunchArgument("port", default_value="8765"),
        OpaqueFunction(function=_bridge),
    ])
