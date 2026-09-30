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
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    target_source = LaunchConfiguration("target_source")
    det_conf = LaunchConfiguration("det_conf")
    reid_engine = LaunchConfiguration("reid_engine")
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
    # YOLOX-S (640 input) since 2026-09-30: on the reid1 recording (894 frames,
    # tools/compare_detectors.py) Nano called a flowering plant a person in 40-67 %
    # of its frames; S and Tiny never did (not even at 0.2), with the same 100 %
    # recall on the operator and higher scores on people 9-11 m away (S 0.86 vs
    # Nano 0.65-0.80). S: 27.7 ms/frame whole pipeline, +4 ms CPU vs Nano.
    # Nano stays available: engine_path:=.../yolox_nano_fp16.engine
    engine_path_arg = DeclareLaunchArgument(
        "engine_path",
        default_value="/home/jetson/ros2_ws/models/yolox_s_fp16.engine",
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
    # ui_server). FOLLOW from the page only with target_source:=tracker.
    use_ui_arg = DeclareLaunchArgument("use_ui", default_value="true")
    # Who publishes /follow/target (exactly one node may):
    #   tracker  -- person_tracker: the person the operator CLICKS in the browser
    #               UI; nothing is followed until someone is selected (default)
    #   selector -- target_selector: automatic "whoever is ahead" (selection_mode);
    #               for solo tests without anyone at the page (follow_solo.sh)
    # Detector confidence threshold. 0.45 = validated default. Lower (e.g. 0.2)
    # also publishes weak detections -- side-on / back views score lower -- which
    # the tracker only uses to EXTEND existing tracks, never to start one.
    det_conf_arg = DeclareLaunchArgument("det_conf", default_value="0.45")
    # Re-ID (appearance) engine: re-identifies the selected person after a loss.
    # "" = off. Built by tools/fetch_model.sh reid.
    reid_engine_arg = DeclareLaunchArgument(
        "reid_engine", default_value="/home/jetson/ros2_ws/models/osnet_x0_25_msmt17_fp16.engine")
    target_source_arg = DeclareLaunchArgument("target_source", default_value="tracker",
                                              choices=["tracker", "selector"])
    is_tracker = PythonExpression(["'", target_source, "' == 'tracker'"])
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
            "conf_threshold": ParameterValue(det_conf, value_type=float),
            "reid_engine": reid_engine,
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
        condition=IfCondition(PythonExpression(
            ["'", use_selector, "' == 'true' and '", target_source, "' == 'selector'"])),
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
            # click mode: search for a lost target and resume if re-identified (30 s)
            "resume_window": ParameterValue(PythonExpression(
                ["30.0 if '", target_source, "' == 'tracker' else 0.0"]), value_type=float),
            "use_sim_time": use_sim_time,
        }],
    )

    # Every person -> a stable ID; the operator's click selects one.
    person_tracker = Node(
        package="dagger_bringup",
        executable="person_tracker",
        name="person_tracker",
        output="screen",
        respawn=True,          # a crash restarts it; the selection is then gone -> no target
        respawn_delay=1.0,
        condition=IfCondition(PythonExpression(
            ["'", use_ui, "' == 'true' or '", target_source, "' == 'tracker'"])),
        parameters=[{
            "image_rotation_deg": image_rotation,
            "publish_target": ParameterValue(is_tracker, value_type=bool),
            "use_sim_time": use_sim_time,
        }],
    )
    # Video + boxes to the browser, clicks back (tornado, latest-frame-only).
    ui_server = Node(
        package="dagger_bringup",
        executable="ui_server",
        name="ui_server",
        output="screen",
        respawn=True,          # a crash restarts it; the selection is then gone -> no target
        respawn_delay=1.0,
        condition=IfCondition(use_ui),
        parameters=[{
            "port": ParameterValue(ui_port, value_type=int),
            "allow_follow": ParameterValue(is_tracker, value_type=bool),
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
        target_source_arg,
        det_conf_arg,
        reid_engine_arg,
        ui_port_arg,
        collision_monitor_follow,
        lifecycle_manager,
        person_detector,
        target_selector,
        follow_controller,
        person_tracker,
        ui_server,
    ])
