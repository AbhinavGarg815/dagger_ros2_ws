import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("dagger_bringup")

    params_file = LaunchConfiguration("params_file")
    extra_params = LaunchConfiguration("extra_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    default_params = os.path.join(bringup_dir, "params", "nav2_dagger.yaml")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
    )
    # Optional overlay applied AFTER params_file (later file wins per-parameter).
    # Defaults to params_file itself (a harmless no-op re-load); the odom-only
    # validation passes nav2_odom_test.yaml here to override global_frame.
    extra_params_arg = DeclareLaunchArgument(
        "extra_params",
        default_value=default_params,
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    # Ground segmentation (P2). true: the costmap's /scan is built from
    # Patchwork++'s NON-GROUND cloud with a 0.12 m floor, so obstacles from ~12 cm
    # up are seen. false: the old fixed 0.35-0.75 m band on the raw cloud, which
    # misses anything under ~35 cm (kerbs, low debris).
    ground_seg = LaunchConfiguration("ground_seg")
    ground_seg_arg = DeclareLaunchArgument("ground_seg", default_value="true")
    # Low-obstacle memory (costmap low_obstacle_layer + low_scan). ON by default since
    # 2026-10-03 (operator OK) after live validation (Q68: walk-by leaves nothing, a low
    # box held to the bumper). v1 was shelved 2026-09-26 (people left blobs, Q32).
    # low_obstacle_memory:=false reverts to the plain costmap.
    low_memory = LaunchConfiguration("low_obstacle_memory")
    low_memory_arg = DeclareLaunchArgument("low_obstacle_memory", default_value="true")
    costmap_cloud = PythonExpression(
        ["'/patchworkpp/nonground' if '", ground_seg, "' == 'true' else '/rslidar_points'"])
    costmap_min_height = PythonExpression(
        ["0.12 if '", ground_seg, "' == 'true' else 0.35"])
    # The low-obstacle layer's inputs. With ground_seg:=false the low band is
    # made empty (floor above ceiling) rather than taken from the raw cloud,
    # where it would be full of ground ring; the layer then stays current and
    # empty instead of going stale and stopping Nav2.
    low_min_height = PythonExpression(["0.12 if '", ground_seg, "' == 'true' else 9.0"])
    low_max_height = PythonExpression(["0.35 if '", ground_seg, "' == 'true' else 8.0"])
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")
    # Near field of the safety scan (P13, report Q65). true: /scan_safety from
    # safety_scan (dagger_perception) -- the rover's own frame removed by a body box,
    # range_min 0.3, so a person beside the hull (inside the 1.03 m circle the corners
    # sweep) is seen. false: p2l_safety with range_min 1.0, blind inside 1.0 m.
    # Default true since 2026-10-03 (operator OK): driven all day with no false stop (Q65-Q68).
    near_field = LaunchConfiguration("near_field")
    near_field_arg = DeclareLaunchArgument("near_field", default_value="true")

    # low_obstacle_memory:=true also ENABLES the costmaps' low_obstacle_layer (off in
    # nav2_dagger.yaml) through this overlay; otherwise extra_params is applied again,
    # which changes nothing.
    low_memory_overlay = PythonExpression(
        ["'" + os.path.join(bringup_dir, "params", "low_obstacle_memory.yaml") + "' if '",
         low_memory, "' == 'true' else '", extra_params, "'"])
    nav2_params = [params_file, extra_params, low_memory_overlay]

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        # Restarted by scan_watchdog if it wedges (lazy-subscribe flap bug, Humble).
        respawn=True,
        respawn_delay=1.0,
        remappings=[
            ("cloud_in", costmap_cloud),
            ("scan", "/scan"),
        ],
        parameters=[{
            "target_frame": "base_footprint",
            "transform_tolerance": 0.1,
            # 0.15 -> 0.35 (2026-09-14): flat GROUND read as +0.19 m in base_footprint
            # (URDF LiDAR height 1.30 m vs ~1.1 m measured from the -15 deg beam's
            # ground hit) -> the lowest-beam ground ring at ~4-5 m became a lethal
            # wall -> Theta* detoured 14 m around a straight 6 m goal. Also margin
            # for outdoor slope/pitch (2 deg at 5 m = 0.17 m). Stopgap until the
            # URDF height is measured + proper ground segmentation replaces this.
            # 0.35 on the raw cloud (ground_seg:=false); 0.12 on Patchwork++'s
            # non-ground cloud, where the ground ring is already removed.
            "min_height": costmap_min_height,
            # 0.90 -> 0.75 (2026-09-14): with the corrected 1.12 m LiDAR height the
            # rover's own mast/mount returns sit at z 0.81-1.12 (r 0.72-0.91 m); at
            # 0.90 they marked a lethal ring INSIDE the footprint -> Spin aborted
            # "Collision Ahead" on the rover itself. Band is now 0.35-0.75 m.
            "max_height": 0.75,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.5,
            "range_max": 30.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
            "use_sim_time": use_sim_time,
        }],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=nav2_params,
        # -> collision_monitor -> /cmd_vel_nav
        remappings=[("cmd_vel", "/cmd_vel_nav_raw")],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=nav2_params,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=nav2_params,
        # -> collision_monitor -> /cmd_vel_nav
        remappings=[("cmd_vel", "/cmd_vel_nav_raw")],
    )

    # Patchwork++ ground segmentation: raw cloud -> /patchworkpp/{ground,nonground}.
    # The node does NOT transform: it only stamps its output with base_frame, so
    # this must be the INPUT cloud's frame or every point is silently mislabelled.
    # min_range 1.0 also drops the mast/mount self-returns (r 0.72-0.91 m).
    # Measured on the July bags: 15.6 ms median, 17 % of one core.
    patchworkpp = Node(
        package="patchworkpp",
        executable="patchworkpp_node",
        name="patchworkpp",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        condition=IfCondition(ground_seg),
        remappings=[("pointcloud_topic", "/rslidar_points")],
        parameters=[{
            "base_frame": "rslidar",
            "sensor_height": 1.12,
            "min_range": 1.0,
            "max_range": 30.0,
            "use_sim_time": use_sim_time,
        }],
    )

    # The OLD band (raw cloud, 0.35-0.75 m) kept as a SECOND costmap source when
    # ground segmentation is on, so segmentation can only ADD obstacles. Replayed
    # data showed it adding ~5 low-obstacle rays per scan but also dropping ~1.7
    # rays the band sees (none within 1.5 m); with both sources that cannot
    # happen, by construction rather than by argument.
    p2l_band = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="p2l_band",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        # Always runs: the costmap lists /scan_band with expected_update_rate,
        # so a missing publisher would mark it stale and stop Nav2 -- which
        # would break ground_seg:=false. There it simply duplicates /scan.
        remappings=[("cloud_in", "/rslidar_points"), ("scan", "/scan_band")],
        parameters=[{
            "target_frame": "base_footprint",
            "transform_tolerance": 0.1,
            "min_height": 0.35,
            "max_height": 0.75,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.5,
            "range_max": 30.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
            "use_sim_time": use_sim_time,
        }],
    )

    # ---- Low-obstacle memory (costmap low_obstacle_layer) -----------------------
    # low_scan (dagger_perception, C++) makes /scan_low from Patchwork++'s non-ground
    # cloud in one pass: low returns (12-35 cm) with nothing tall (40-160 cm) within
    # 0.5 m, and only once their odom cell has been hit in >= 5 scans spanning >= 1.0 s
    # (people move, boxes don't; a standing person's feet sit next to their body). The layer clears only beyond 4 m, so a low
    # object seen from afar is remembered while the rover closes in (Q68: without it
    # a 26 cm box was erased 3.8 m out and driven over). Replaces two extra p2l
    # instances and the Python low_scan_filter (2026-10-03).
    low_scan = Node(
        package="dagger_perception",
        executable="low_scan",
        name="low_scan",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        condition=IfCondition(low_memory),
        remappings=[("cloud_in", costmap_cloud), ("odom", "/odometry/filtered"),
                    ("scan", "/scan_low")],
        parameters=[{
            "target_frame": "base_footprint",
            "low_min_height": low_min_height,
            "low_max_height": low_max_height,
            "tall_min_height": 0.40,
            "tall_max_height": 1.60,
            "tall_radius": 0.5,     # anything tall this close: part of a tall thing
            "range_min": 0.5,
            "range_max": 30.0,
            "persistence": True,
            "cell_size": 0.15,
            "persist_window": 2.0,
            "min_hits": 5,
            "min_span": 1.0,
            "body_box": [-0.90, 0.50, -0.60, 0.60],
            "use_sim_time": use_sim_time,
        }],
    )

    # Permanent /scan subscriber (prevents the p2l lazy-subscribe flap) + restarts
    # p2l if /scan goes silent while clouds still arrive.
    scan_watchdog = Node(
        package="dagger_bringup",
        executable="scan_watchdog",
        name="scan_watchdog",
        output="screen",
        # match THIS p2l instance only (p2l_safety runs the same executable).
        # Watch p2l's ACTUAL input: with ground segmentation on, blaming p2l for a
        # silent /scan while the raw cloud flows would restart the wrong node.
        parameters=[{"process_pattern": "__node:=pointcloud_to_laserscan( |$)",
                     "cloud_topic": costmap_cloud}],
    )

    # ---- Collision monitor: independent last gate on the Nav2 command path ----
    # Dedicated scan for the safety layer, separate from the costmap /scan: taller
    # band (0.30-1.60 m, a person is seen right up to the bumper). Two sources,
    # chosen by near_field, with the same band, bins and range_max:
    #  safety_scan -- drops the rover's own top frame and GNSS posts with a body box
    #    in base_footprint (all of it lies inside x -0.85..0.42, y +-0.56; Q65),
    #    then range_min 0.3. Bin for bin identical to p2l with the box off.
    #  p2l_safety -- range_min 1.0 m to drop those returns (r <= 0.96 m), which also
    #    hides everything else within 1.0 m of the centre.
    safety_scan = Node(
        package="dagger_perception",
        executable="safety_scan",
        name="safety_scan",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        condition=IfCondition(near_field),
        remappings=[("cloud_in", "/rslidar_points"), ("scan", "/scan_safety")],
        parameters=[{
            "target_frame": "base_footprint",
            "min_height": 0.30,
            "max_height": 1.60,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.3,
            "range_max": 20.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
            # x_min, x_max, y_min, y_max (m, base_footprint, all heights). Margins >= 4 cm
            # over the frame; the front deck (x 0.50-0.85) stays visible. Re-check with
            # the empty capture (COMMANDS.md section 8) after mounting anything on top.
            "body_filter": True,
            "body_box": [-0.90, 0.50, -0.60, 0.60],
            "use_sim_time": use_sim_time,
        }],
    )

    p2l_safety = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="p2l_safety",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        condition=UnlessCondition(near_field),
        remappings=[("cloud_in", "/rslidar_points"), ("scan", "/scan_safety")],
        parameters=[{
            "target_frame": "base_footprint",
            "transform_tolerance": 0.1,
            "min_height": 0.30,
            "max_height": 1.60,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 1.0,
            # 10 -> 20 m (2026-09-26): this scan is also the follow-me range and
            # LiDAR-hold source, and a person beyond 10 m simply had no range at
            # all. The collision monitor only looks inside ~2.6 m, so the longer
            # range costs it nothing.
            "range_max": 20.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
            "use_sim_time": use_sim_time,
        }],
    )

    scan_safety_watchdog = Node(
        package="dagger_bringup",
        executable="scan_watchdog",
        name="scan_safety_watchdog",
        output="screen",
        parameters=[{
            "scan_topic": "/scan_safety",
            "process_pattern": PythonExpression(
                ["'__node:=safety_scan( |$)' if '", near_field,
                 "' == 'true' else '__node:=p2l_safety( |$)'"]),
        }],
    )

    # /cmd_vel_nav_raw -> stop / slowdown / approach -> /cmd_vel_nav (twist_mux).
    # Lifecycle-managed with Nav2: if it cannot activate, Nav2 does not come up.
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            os.path.join(bringup_dir, "params", "collision_monitor.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        # Dagger trees: default Nav2 trees minus the Spin/BackUp motion recoveries.
        parameters=nav2_params + [{
            "default_nav_to_pose_bt_xml": os.path.join(
                bringup_dir, "behavior_trees", "dagger_navigate_to_pose.xml"),
            "default_nav_through_poses_bt_xml": os.path.join(
                bringup_dir, "behavior_trees", "dagger_navigate_through_poses.xml"),
        }],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "collision_monitor",
            ],
        }],
    )

    # GUIDED setpoint-velocity command path (NO RC override): safety.launch.py ->
    # cmd_vel_safety -> /mavros/setpoint_velocity/cmd_vel_unstamped. Requires the
    # mavros plugin param mav_frame=BODY_NED so the twist is interpreted in the
    # body frame (as Nav2 emits it). NOTE: ArduPilot's EKF yaw now comes from the
    # dual-antenna C-RTK 2HP (EK3_SRC1_YAW=2), so the old compassless-GSF drift and
    # the ExternalNav yaw relay that worked around it are history (relay kept
    # arg-gated off in localization.launch.py).
    command_path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "safety.launch.py")
        ),
        # cmd_vel_safety zeroes output if /scan_safety goes stale (Humble's
        # collision_monitor would otherwise treat a dead source as "all clear").
        launch_arguments={"fresh_topic": "/scan_safety"}.items(),
    )

    return LaunchDescription([
        params_file_arg,
        extra_params_arg,
        use_sim_time_arg,
        autostart_arg,
        ground_seg_arg,
        low_memory_arg,
        near_field_arg,
        patchworkpp,
        p2l_band,
        low_scan,
        pointcloud_to_laserscan,
        scan_watchdog,
        safety_scan,
        p2l_safety,
        scan_safety_watchdog,
        collision_monitor,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
        command_path,
    ])
