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

The detector runs here too, behind use_detector. The target selector and follow
controller land here later, publishing /cmd_vel_follow_raw into this monitor.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("dagger_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    use_detector = LaunchConfiguration("use_detector")
    use_selector = LaunchConfiguration("use_selector")
    selection_mode = LaunchConfiguration("selection_mode")
    use_controller = LaunchConfiguration("use_controller")
    follow_distance = LaunchConfiguration("follow_distance")
    max_linear = LaunchConfiguration("max_linear")
    engine_path = LaunchConfiguration("engine_path")
    debug_image = LaunchConfiguration("debug_image")
    image_rotation = LaunchConfiguration("image_rotation_deg")
    use_ui = LaunchConfiguration("use_ui")
    ui_port = LaunchConfiguration("ui_port")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")
    use_detector_arg = DeclareLaunchArgument("use_detector", default_value="true")
    use_selector_arg = DeclareLaunchArgument("use_selector", default_value="true")
    use_controller_arg = DeclareLaunchArgument("use_controller", default_value="true")
    # "centred" follows whoever is ahead; "nearest" grabs the closest person.
    selection_mode_arg = DeclareLaunchArgument("selection_mode", default_value="centred")
    follow_distance_arg = DeclareLaunchArgument("follow_distance", default_value="3.0")
    # Forward speed ceiling. 0.6 validated on 2026-09-26: cruises at ~0.57 m/s,
    # stops in 0.286 m from 0.60-0.63 m/s (0.31 m inside the 0.6 m stop zone),
    # no real overshoot. (0.4: 0.165 m from 0.384 m/s.) Needs WP_SPEED >= 0.6 on
    # the FCU -- ArduPilot Rover caps GUIDED velocity at WP_SPEED, which was
    # holding every earlier run near 0.4. Measure the stop before going higher.
    max_linear_arg = DeclareLaunchArgument("max_linear", default_value="0.6")
    # Engines are built for one GPU + TensorRT version, so models/ is gitignored
    # and this points into the source tree rather than the install share dir.
    engine_path_arg = DeclareLaunchArgument(
        "engine_path",
        default_value="/home/jetson/ros2_ws/models/yolox_nano_fp16.engine",
        description="TensorRT engine from tools/fetch_model.sh",
    )
    # Annotated frames for Foxglove. Off while driving -- Foxglove already
    # starves the controller, and this adds an encode plus a publish per frame.
    debug_image_arg = DeclareLaunchArgument("debug_image", default_value="false")
    # The D435 on this rover is mounted upside down, so the detector rotates
    # frames 180 deg before inference (YOLOX is not rotation invariant). Keep
    # this in step with the roll on dagger_description's d435_joint; the URDF
    # fixes the GEOMETRY, this fixes what the network is shown.
    image_rotation_arg = DeclareLaunchArgument("image_rotation_deg", default_value="180")
    # Click-to-select browser UI: http://<rover-ip>:<ui_port>/ (person_tracker +
    # ui_server). Following from the page is off until the tracker drives
    # /follow/target from the selection (phase 2).
    use_ui_arg = DeclareLaunchArgument("use_ui", default_value="true")
    ui_port_arg = DeclareLaunchArgument("ui_port", default_value="8080")

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

    # Colour frames -> /follow/detections. Throttled to 10 Hz to match the LiDAR;
    # the detector itself measured 66.7 FPS, so the limit is deliberate headroom.
    person_detector = Node(
        package="dagger_bringup",
        executable="person_detector",
        name="person_detector",
        output="screen",
        condition=IfCondition(use_detector),
        parameters=[{
            "engine_path": engine_path,
            "target_rate": 10.0,
            "publish_debug_image": debug_image,
            "image_rotation_deg": image_rotation,
            "use_sim_time": use_sim_time,
        }],
    )

    # Bearing from the camera, range from the LiDAR -> /follow/target.
    target_selector = Node(
        package="dagger_bringup",
        executable="target_selector",
        name="target_selector",
        output="screen",
        condition=IfCondition(use_selector),
        parameters=[{
            "selection_mode": selection_mode,
            "use_sim_time": use_sim_time,
        }],
    )

    # /follow/target -> /cmd_vel_follow_raw. Starts DISABLED: launching the
    # follow stack must never be what makes the rover move. The operator enables
    # it by publishing true on /follow/enable.
    follow_controller = Node(
        package="dagger_bringup",
        executable="follow_controller",
        name="follow_controller",
        output="screen",
        condition=IfCondition(use_controller),
        parameters=[{
            "follow_distance": follow_distance,
            "max_linear": max_linear,
            "use_sim_time": use_sim_time,
        }],
    )

    # Every person -> a stable ID; the operator's click selects one.
    person_tracker = Node(
        package="dagger_bringup",
        executable="person_tracker",
        name="person_tracker",
        output="screen",
        condition=IfCondition(use_ui),
        parameters=[{
            "image_rotation_deg": image_rotation,
            "use_sim_time": use_sim_time,
        }],
    )
    # Video + boxes to the browser, clicks back (tornado, latest-frame-only).
    ui_server = Node(
        package="dagger_bringup",
        executable="ui_server",
        name="ui_server",
        output="screen",
        condition=IfCondition(use_ui),
        parameters=[{
            "port": ParameterValue(ui_port, value_type=int),
            "allow_follow": False,
            "use_sim_time": use_sim_time,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        use_detector_arg,
        use_selector_arg,
        use_controller_arg,
        selection_mode_arg,
        follow_distance_arg,
        max_linear_arg,
        engine_path_arg,
        debug_image_arg,
        image_rotation_arg,
        use_ui_arg,
        ui_port_arg,
        collision_monitor_follow,
        lifecycle_manager,
        person_detector,
        target_selector,
        follow_controller,
        person_tracker,
        ui_server,
    ])
