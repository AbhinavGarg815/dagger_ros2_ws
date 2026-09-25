"""Intel RealSense D435 -- COLOUR ONLY, for the follow-me detector.

    ros2 launch dagger_bringup camera.launch.py
    ros2 launch dagger_bringup drive_bringup.launch.py use_camera:=true   # with T1

Publishes /d435/color/image_raw and /d435/color/camera_info.

THE CAMERA IS A D435, NOT A D435i -- there is no IMU on it, whatever it gets
called. Nothing here consumes camera motion data.

DEPTH IS DELIBERATELY OFF. The D435's stereo depth washes out in direct
sunlight, and this rover is outdoor-only; range for the follow controller comes
from the LiDAR via /scan_safety, not from the camera. Leaving depth, infra and
the pointcloud filter off also keeps the USB bandwidth and the CPU cost down --
Nav2 already starves at times on the 6-core Orin.

Explicit colour profile rather than the driver's "0,0,0" auto-pick, so the frame
rate and resolution the detector was benchmarked against cannot silently change.
640x480 is well above YOLOX's 416x416 input.

TF: the driver publishes its own internal static transforms (d435_link ->
d435_color_optical_frame etc.), but NOTHING connects d435_link to the robot --
dagger_description has no camera link yet. Until it does, a detection bearing
cannot be transformed into the LiDAR's frame. Needs a measured mount position
(x forward, y left, z above ground, and the aim), same as the LiDAR height did.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    color_profile = LaunchConfiguration("color_profile")
    initial_reset = LaunchConfiguration("initial_reset")

    color_profile_arg = DeclareLaunchArgument(
        "color_profile", default_value="640,480,30", description="width,height,fps"
    )
    # A D435 left in a bad state by an unclean shutdown can enumerate but never
    # stream. initial_reset:=true power-cycles it at start-up; it costs a couple
    # of seconds, so it is opt-in rather than the default.
    initial_reset_arg = DeclareLaunchArgument("initial_reset", default_value="false")

    # namespace MUST be set to "/" explicitly. The driver publishes on PRIVATE
    # topics (~/color/image_raw) AND realsense_node_factory.cpp:35 constructs the
    # node with a hardcoded "/camera" default namespace, so the topic ends up at
    # <namespace>/<node name>/color/image_raw:
    #     namespace unset,   name d435 -> /camera/d435/color/image_raw
    #     namespace "d435",  name d435 -> /d435/d435/color/image_raw
    #     namespace "/",     name d435 -> /d435/color/image_raw   <- what we want
    # (this is also why the stock rs_launch.py produces /camera/camera/...).
    d435 = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="/",
        name="d435",
        output="screen",
        parameters=[{
            "camera_name": "d435",
            "enable_color": True,
            "rgb_camera.color_profile": color_profile,
            "enable_depth": False,
            "enable_infra": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "pointcloud.enable": False,
            "enable_sync": False,
            "initial_reset": initial_reset,
        }],
    )

    return LaunchDescription([color_profile_arg, initial_reset_arg, d435])
