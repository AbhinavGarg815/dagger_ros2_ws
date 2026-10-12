# Dagger UGV — command reference

Every command used to build, run, test and debug the rover, in the order you need them.
Last updated: 2026-10-12 (operator-app gateway `dagger_gateway` on :8090; `fcu_config` in T1; systemd units in `tools/systemd/`).

**Conventions**

- Run everything from the workspace, in a terminal that has the environment loaded:
  ```bash
  cd /home/jetson/ros2_ws
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ```
- Tags: **[MOTION]** the rover may move — operator runs it, RC transmitter in hand ·
  **[SUDO]** needs root · everything else only reads or configures.
- `T1`…`T6` are the terminals of the running stack (section 3). Start them in that order.
- Bags go to `~/dagger_bags/`. Stop the recorder (Ctrl-C) before reading a bag.

---

## 1. One-time setup

| Command | Use |
|---|---|
| `sudo bash tools/setup_host.sh` | **[SUDO]** Kernel network buffers for the LiDAR UDP stream and DDS. Survives reboots. |
| `vcs import src < dagger.repos` | Fetch the third-party packages at their pinned commits (LiDAR driver, RealSense, KISS-ICP, …). |
| `bash tools/fetch_patchworkpp.sh` | Fetch and build Patchwork++ (ground segmentation) at its pinned commit. |
| `bash tools/fetch_model.sh yolox_s` | Download the person detector and build its TensorRT engine for this Jetson. Also: `yolox_nano` (default), `reid` (OSNet re-identification). |
| `sudo apt install python3-tornado` | **[SUDO]** Web server for the follow page. |
| `sudo apt install protobuf-compiler` | **[SUDO]** protoc 3.12.4 for the gateway (matches apt `python3-protobuf`). |
| `git submodule update --init proto` | The app ↔ rover contract (`dagger_proto`) at `proto/`. |
| `sudo bash tools/systemd/install.sh` | **[SUDO]** Install + enable the boot services (section 3); `--remove` undoes it. |

Engines in `models/` are tied to this GPU and TensorRT version: rebuild them after a JetPack update.

## 2. Build

| Command | Use |
|---|---|
| `colcon build --packages-select dagger_bringup` | Rebuild our nodes, launch files, params, web page (seconds). |
| `colcon build --packages-select dagger_msgs dagger_bringup` | After changing a message definition (about 1 min). |
| `colcon build --packages-select dagger_description` | After changing the URDF or its launch files. |
| `colcon build --packages-select dagger_perception` | After changing `safety_scan` / `low_scan` (C++, about 50 s). |
| `colcon build --packages-select dagger_nav_plugins` | After changing the Nav2 plugins (StickyGoalChecker, about 40 s). |
| `colcon build --packages-select dagger_gateway` | After changing the gateway or `proto/` (regenerates `api_pb2.py`, seconds). |

A running node keeps its old code: restart the terminal that runs it after a build.
The install is not symlinked — an edit in `src/` does nothing until built.

## 3. Bring-up

| Terminal | Command | What it starts |
|---|---|---|
| T1 | `ros2 launch dagger_bringup drive_bringup.launch.py` | FCU link (mavros), LiDAR, robot model. Add `use_camera:=true` for following. |
| T2 | `ros2 launch dagger_bringup nav2_gps.launch.py` | Odometry, GPS map frame, Nav2, costmaps, collision monitor, command path to the FCU. Add `map_source:=slam` when T5 localizes. On by default: the near-field safety scan (sees a person beside the hull), the low-obstacle memory (low obstacles seen from afar stay in the costmaps) and the overhang layer (canopies and branches 0.75–1.35 m in the costmaps); `near_field:=false` / `low_obstacle_memory:=false` / `overhang:=false` revert each. LiDAR odometry fused into `/odometry/filtered` by default (Q75): `lidar_odom:=shadow` runs it without fusing, `lidar_odom:=false` reverts to the FCU velocity alone. With it, `extnav_relay` feeds the Cube ExternalNav; **RC10 middle** = indoor (EK3 source set 2, LiDAR), **low** = GPS (Q80). Indoors: `python3 tools/preflight.py --indoor`, odom-frame goals only. **Waypoint routes (T3):** Foxglove pose tool topic `/waypoint_ui` → click waypoints (drawn on `/waypoints/path`) → Publish panel `/waypoints/go` (`std_msgs/msg/Empty`, `{}`); `/waypoints/clear` to start over. Via points pass within 2 m; the last one's heading applies. Things entirely above 1.35 m (above the rover) are ignored by the stop path: `SAFETY_BAND_TOP` in nav2.launch.py — raise it if anything taller is mounted. |
| T3 | `ros2 launch dagger_bringup foxglove.launch.py` | Foxglove bridge on `ws://<rover-ip>:8765`. Add `with_cloud:=true` for the raw point cloud (not while driving). |
| T4 | `ros2 launch dagger_bringup follow.launch.py …` | Person / marker following and its web page (section 7). |
| T5 | `ros2 launch dagger_bringup slam.launch.py …` | 2D SLAM (section 6). |
| T6 | `ros2 run dagger_gateway gateway` | Operator-app gateway, `ws://<rover-ip>:8090/api` (protobuf, contract in `proto/`). `--stop-dry-run`: STOP changes the state only. |

T1 also runs `fcu_config`: `mav_frame BODY_NED` + stream rates re-applied on every FCU connect (Q83).

**At boot (systemd, once installed):** `dagger.target` = T1 (`use_camera:=true`) → T2 → T4 (people + marker) → gateway;
T3 and T5 stay manual. Don't start those terminals by hand while the services run.

```bash
sudo systemctl stop dagger.target      # back to terminals     · sudo systemctl start dagger.target
systemctl status 'dagger-*'            # what runs             · journalctl -fu dagger-gateway   (or -nav, -drive, -follow)
```

Then, **before any drive and again after every T1 or mavros restart**:

```bash
python3 tools/preflight.py            # configure the FCU link + health check: must end in PASS
                                      # (incl. GNSS heading valid >= 90 % and FCU estimator still while parked)
python3 tools/preflight.py --no-config   # check only
python3 tools/preflight.py --no-nav2     # T1 only
python3 tools/preflight.py -t 60         # 60 s sample: stationary pose-noise baseline
```

Arming and the GUIDED mode are set by the operator (RC transmitter / Mission Planner).
`hostname -I` prints the rover's addresses (Wi-Fi was `10.193.159.72` on 2026-09-30).

## 4. Stop

| Command | Use |
|---|---|
| RC transmitter | Primary stop. Always in hand when the rover can move. |
| Web page: **STOP** button or `space` | Follow off. `esc` / **CLEAR** deselects the target (also ends following). |
| `bash tools/estop.sh` | FCU to HOLD and `/estop` latched at 10 Hz. Leave it running. |
| Operator app: **STOP** | Same as `estop.sh` (HOLD + `/estop` latched) plus Nav2 goals cancelled and follow off. Any `/estop true` shows the app STOPPED. |
| `bash tools/estop_clear.sh` | Release the E-stop (also the app's STOP: the gateway stops streaming). The mode stays HOLD: re-select GUIDED yourself. |
| `ros2 topic pub -t 10 -r 5 /follow/enable std_msgs/Bool "{data: false}"` | Follow off from a terminal (repeated: a single message can be dropped). |
| `ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"` | Cancel the Nav2 goal. Ctrl-C in `send_goal.py` does the same. |
| `ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"` | Disarm. |

## 5. Navigation (Nav2)

Needs T1 + T2 and a preflight PASS.

| Command | Use |
|---|---|
| `python3 tools/route_check.py` | Free space ahead, from the live costmap. With `"3,0 6,0.5"` it scores that route. |
| `python3 tools/send_goal.py 5` | **[MOTION]** Goal 5 m straight ahead. `--left 1` adds 1 m left; `--dyaw 90` turns; `--wp "5,0 10,0 10,4"` waypoints (forward,left from the start pose). |
| `python3 tools/gps_goal.py --mission --dry-run` | FCU mission converted to map coordinates, nothing sent. |
| `python3 tools/gps_goal.py --mission` | **[MOTION]** Drive the FCU mission's waypoints. |
| `python3 tools/gps_goal.py --ll "26.74798,75.85534 26.74801,75.85540"` | **[MOTION]** Lat/lon waypoints. `--enu "8,0 8,5"` = metres east,north of the rover. In SLAM mode add `--anchor geo` (default) or `--anchor live`. |
| Foxglove 3D panel (display + fixed frame **`odom`**) → publish **Pose** on `/goal_pose_ui` | **[MOTION]** Click-to-goal, planned around obstacles. Not `map` without RTK: map→odom follows the GNSS walk and the goal slides. Never click near drop-offs (Nav2 cannot see them). **Pose estimate** (frame `map`) on `/initialpose_ui` re-seeds SLAM localization. |
| `bash tools/solo_goal.sh <bag> "8" 15:20:00 15:22:00` | **[MOTION]** Solo test: at each clock slot, if the safety gate passes, records and sends the goal. |
| `python3 tools/solo_gate.py --save` | Remember the start pose for the solo gate (`solo_gate.py` alone = check it). |
| `bash tools/pf_tune.sh show` | Current path-following parameters. `tuned` / `base` apply a profile live (lost at T2 restart). |

## 6. SLAM

**Mapping** (runs beside everything, publishes nothing into the real TF tree):

```bash
ros2 launch dagger_bringup slam.launch.py                     # T5, shadow mapping
bash tools/slam_record.sh <name>                              # record the drive (add `clouds` for raw point clouds)
# save, while T5 is still running:
ros2 run nav2_map_server map_saver_cli -t /slam/map -f <path> --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=20.0
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '<path>'}"
# tie the map to GPS (needed for lat/lon goals and the automatic start pose):
python3 tools/slam_georef.py <bag> <poses.csv> src/dagger_bringup/maps/<name>_geo.yaml   # fixes weighted 1/sigma^2, sigma > 8 m dropped (--max-sigma): indoor parts of a drive don't pull the anchor
```

**Localization** on a saved map (`src/dagger_bringup/maps/<name>.*`):

```bash
ros2 launch dagger_bringup nav2_gps.launch.py map_source:=slam               # T2
ros2 launch dagger_bringup slam.launch.py mode:=localization map:=lane3fix   # T5
python3 tools/preflight.py                                                   # checks who owns map->odom
```
Back to GPS: T2 without `map_source`, T5 without `mode`.

**Offline** (own ROS domain, no rover):

| Command | Use |
|---|---|
| `bash tools/slam_replay.sh <bag> <label> [param:=value …]` | Re-map a recorded drive with other settings → `~/dagger_bags/replay/<label>/`. |
| `bash tools/slam_loc_replay.sh <bag> <label> <map> [off_x off_y]` | Test localization against a saved map; the offsets shift the GPS start to test recovery. |
| `python3 tools/slam_replay_eval.py <bag> <label> …` | Score mapping replays. `--loc <label> … [<bag>]` scores localization replays. |

## 7. Following (people and ArUco markers)

Needs T1 with `use_camera:=true`, T2, and a preflight PASS before driving.

```bash
# people only
ros2 launch dagger_bringup follow.launch.py det_conf:=0.2
# people + marker (our marker: DICT_6X6_50, id 0, 17.5 cm)
ros2 launch dagger_bringup follow.launch.py det_conf:=0.2 marker_dictionary:=DICT_6X6_50 marker_ids:=0 marker_size:=0.175
# markers only (person detector not loaded)
ros2 launch dagger_bringup follow.launch.py detect_people:=false marker_dictionary:=DICT_6X6_50 marker_ids:=0 marker_size:=0.175
```

Open the page, click (tap) a person or marker, press **FOLLOW** **[MOTION]**:

- laptop / phone on Wi-Fi: `http://<rover-ip>:8080`
- **SIYI MK32 handset** (over the SIYI radio): `http://192.168.144.100:8080` in its browser, then **FULL**.
  Touch screens get the handset layout (big buttons on the right); force with `?handset=1` / `?handset=0`.
- **LITE** button, or `?video=lite`: half-size video (about 10 KB/frame instead of 29) for a slow, long-range link.
- The page that pressed FOLLOW is the deadman: if it goes silent for 1.5 s or disconnects, follow stops,
  whatever other pages are open.

The rover keeps 3 m (`follow_distance:=`), at most 0.6 m/s (`max_linear:=`), stands still while the
target is within 1.6 m, and searches by turning for 30 s if it loses the target.

| Command | Use |
|---|---|
| `python3 tools/marker_probe.py --size 0.175` | Identify a marker: waits until it is held in view, then prints dictionary, id, size check and the launch line. `--wait 600` waits longer. |
| `python3 tools/follow_check.py` | Wiring check of the follow safety path (collision monitor active, scan fresh, single command path). |
| `python3 tools/follow_status.py` | One line per tick for the whole chain: detections, target, command, gate. |
| `ros2 launch dagger_bringup follow.launch.py target_source:=selector` | Old automatic mode: follows whoever is ahead, no page. |
| `bash tools/follow_solo.sh <bag> 14:05:00 14:07:00` | **[MOTION]** Solo test of the automatic mode: enables follow at a clock slot if the gate passes. |

## 8. Recording

| Command | Records | Read with |
|---|---|---|
| `bash tools/pf_record.sh <name>` | Path following and follow commands, odometry, FCU I/O, TF, logs. No point clouds. | `pf_analyze.py`, `stop_distance.py` |
| `bash tools/track_record.sh <name>` | Everything the people / marker trackers see and output, the page's video, follow commands. | `replay_tracker.py`, `compare_detectors.py` |
| `bash tools/slam_record.sh <name> [clouds]` | Scans, odometry, GPS, TF for re-mapping. | `slam_replay.sh` |
| `bash tools/detour_record.sh <name>` | Obstacle approach / detour tests: raw cloud, Patchwork++ output, all scans, both costmaps, plan, commands (~7 MB/s). | offline replay of the low-obstacle pipeline |
| `bash record_drive.sh` | Full drive including raw point clouds (large). | `test_ground_seg.py` |
| `timeout -s INT 40 ros2 bag record -s mcap -o ~/dagger_bags/body_empty_$(date +%H%M%S) /rslidar_points /tf_static /scan_safety /odometry/filtered` | Parked, disarmed, nothing within 2 m: the rover's own LiDAR returns. Repeat after mounting anything on top. | `test_safety_scan.py --empty <bag>` |

Names are timestamped automatically; start the recorder before the test and stop it with Ctrl-C.

## 9. Analysis

| Command | Use |
|---|---|
| `python3 tools/pf_analyze.py <bag> [--png out.png]` | Path following: cross-track and heading error, commanded vs actual turn rate, goal error. |
| `python3 tools/stop_distance.py <bag> …` | Time and distance from the stop command to rest. |
| `python3 tools/replay_tracker.py <bag> --reid [--select-at 30] [--why] [--set k=v]` | Re-run the person tracker offline on a recording; `--live` scores what the live tracker did; `--why` prints each LiDAR-hold refusal. |
| `python3 tools/compare_detectors.py <bag> --ref 4:you --ref 8:plant` | Compare detector models on a recording against labelled tracks. |
| `python3 tools/detect_bench.py` | Detector sanity check and speed on this Jetson. |
| `python3 tools/node_cpu.py <name> … [-i 5 -d 600 -o out.csv]` | CPU of the named processes (one sample, or logged over time). |
| `python3 tools/ui_latency.py [--lite] [-s 20]` | Delay of the page's video leaving the rover (camera → sent) and of its boxes (camera → boxes), frame size and rate. Needs T1 with camera + T4. |

## 10. Debugging

| Symptom / question | Command |
|---|---|
| Is the FCU connected, armed, which mode? | `ros2 topic echo /mavros/state --once` |
| Is FCU telemetry really arriving? | `python3 tools/fcu_flowing.py` |
| Is the FCU estimator healthy (GNSS heading, no drift)? | `python3 tools/preflight.py --no-config` (FCU / GNSS section). The FCU's own messages (`FCU: AHRS: DCM active` ↔ `EKF3 active` flipping = EKF3 unhealthy) appear only in the T1 terminal. A full power-restart of the rover cleared one such divergence (2026-10-03). |
| Is a sensor or topic alive, at what rate? | `ros2 topic hz /rslidar_points` · `/scan` · `/scan_safety` · `/scan_overhang` · `/odometry/filtered` · `/d435/color/image_raw` |
| What is following doing? | `ros2 topic echo /follow/state` · `python3 tools/follow_status.py` |
| Who publishes / subscribes a topic? | `ros2 topic info -v <topic>` |
| Is a lifecycle node active? | `ros2 lifecycle get /collision_monitor_follow` (or `/controller_server`, `/planner_server`, …) |
| Is a transform there, who owns it? | `ros2 run tf2_ros tf2_echo map odom` |
| A node's parameter | `ros2 param get <node> <name>` · `ros2 param list <node>` |
| Obstacles in the costmap that are not there | `python3 tools/costmap_audit.py [local\|global]` |
| Does the costmap see a low object ahead? | `python3 tools/scan_ahead.py` · `python3 tools/low_obstacle_approach.py` |
| Does a collision monitor stop / slow / cut a turn? (**disarmed only**, it refuses otherwise) | `python3 tools/follow_gate_test.py --path nav` (or `follow`) — walk in ahead; `--vx 0 --wz 0.45`: turn in place, stand beside the hull; `--expect-clear -t 600`: nobody near, no false trigger |
| Which nodes are up? | `ros2 node list` |
| Is the SIYI handset reachable? | `ping 192.168.144.20` (about 5 ms round trip over the radio) |
| `ros2` commands hang or list stale nodes | `ros2 daemon stop; ros2 daemon start` |
| Node logs (newest first) | `ls -t ~/.ros/log \| head` then `grep -i "error\|warn" ~/.ros/log/<file>` |
| Stop one stuck process | `pgrep -af <name>` then `kill <PID>` — never `pkill -f <pattern>` (it has matched the wrong shell) |
| CPU load | `uptime` · `python3 tools/node_cpu.py <name>` |

Preflight sets two things that reset on every mavros restart — velocity frame BODY_NED and the
MAVLink stream rates. If the rover ignores commands after a T1 restart, rerun preflight.

## 11. Offline tests

No rover needed; they run in their own ROS domain. Stop T4 first — and preferably the whole
stack — because the scene tests are timing-sensitive under load.

| Command | Expect | Covers |
|---|---|---|
| `python3 tools/test_person_tracking.py` | 64/64 | Person tracker rules |
| `python3 tools/test_marker_tracking.py` | 43/43 | Marker geometry, detection on real frames, tracker rules |
| `python3 tools/test_nav_slow_zone.py` | 11/11 | Nav2 slow zone: x0.4 while driving near obstacles, in-place turns untouched (the 0.12 rad/s turn start passes) |
| `python3 tools/test_low_scan.py` | 13/13 | Low-obstacle memory: static box confirmed, people (standing, crouching) / walkers / no odometry never; replay of the 10-03 low-box run keeps the box to the bumper, nothing else in the lane. `--replay <bag> [--replay-only] [--params "-p min_span:=1.0"]` for another recording (box checks then do not apply). |
| `python3 tools/test_estimator_checks.py` | 9/9 | preflight GNSS-heading / FCU-estimator checks and the odometry relay's speed gate, against a fake FCU |
| `python3 tools/test_safety_scan.py` | 33/33 | `/scan_safety` + `/scan_overhang` (same node, live params): body box, bands 0.30–1.35 / 0.75–1.35, bins; identical to p2l on the 10-02 captures; parked + walk-around captures; fox2 canopy no longer stops, Q66 plant reaches the overhang scan |
| `python3 tools/test_lidar_odom.py` | 12/12 | `lidar_odom` (dagger_perception): synthetic room/motion/rejects/corridor + lockstep replay of fox3 vs a direct registration |
| `python3 tools/test_extnav_relay.py` | 5/5 | extnav_relay (LiDAR odometry → ArduPilot ExternalNav, Q80) + mavros vision plugins load and subscribe (no FCU) |
| `python3 src/dagger_gateway/test/test_vectors.py` | 19 vectors | Gateway bytes: every `proto/vectors/*.bin` decodes to its `.txtpb` and re-encodes identically |
| `python3 src/dagger_gateway/test/test_rover_state.py` | 7/7 | Gateway state machine, alerts, link-loss policy (incl. route pause) |
| `python3 src/dagger_gateway/test/test_gateway_ros.py` | 44/44 | Gateway vs a fake rover: telemetry, readiness, follow-off on app silence, real STOP + estop_clear release |
| `python3 proto/contract_test.py --url ws://127.0.0.1:8090/api` | 29/29 | The app's contract against a running gateway (sends STOP: dry run, or rover parked + RC in hand). From the PC: `--url ws://192.168.144.100:8090/api --allow-real-rover` |
| `python3 tools/test_fcu_config.py` | PASS | `fcu_config`: BODY_NED + stream rates on connect and reconnect (fake mavros) |
| `python3 tools/test_waypoint_queue.py` | 7/7 | waypoint_queue (Foxglove clicks → NavigateThroughPoses, odom tree) against a stub Nav2 |
| `colcon test --packages-select dagger_nav_plugins && colcon test-result --verbose --test-result-base build/dagger_nav_plugins` | 4/4 | StickyGoalChecker (Q76): the live hunting case, new goal, drift re-arm, plain goals |
| `python tools/lo_bench.py <bag> <out.npz> [methods]` then `python tools/lo_eval.py <out.npz>` | — | LiDAR-odometry benchmark (KISS, GenZ, VGICP variants) without ground truth; needs a venv with the pip wheels `kiss-icp` 1.3.0, `small_gicp` 1.0.1, GenZ-ICP built from source, and `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1` |
| `python tools/lo_node_bench.py <bag> <out.npz> [--domain 86]` then `python tools/lo_eval.py <out.npz>` | — | the C++ `lidar_odom` node replayed in lockstep beside the Python VGICP reference, scored by the same evaluator (same venv) |
| `python tools/lo_bag_to_bench.py <bag> <out.npz>` then `python tools/lo_eval.py <out.npz>` | — | a recorded drive with `lidar_odom` running: live `/lidar_odom/odometry` vs `/odometry/filtered` against direct registrations; health + start→end check (same venv) |
| `python3 tools/session_summary.py <bag>` | — | a goal session at a glance: FCU states/messages, goals (status, distance, heading), in-place turns, collision monitor effect, warnings |
| `python3 tools/ekf_replay.py <bag> <out_dir> [--variants V0,V1,..] [--rate 2]` then `python tools/ekf_replay_score.py <bench.npz> <out_dir> <merged.npz>` and `python tools/lo_eval.py <merged.npz>` | — | replay `ekf_local` offline with input variants (heading sources, LiDAR yaw rate/pose) from a bag recorded since Q75; scored like the odometries (Q77) |
| `python3 tools/test_follow_controller.py` | 26/26 | Follow controller: speeds, search, close-range hold, CLEAR |
| `python3 tools/test_click_follow.py all` | 53/53 | Person following scenes (or one of `main cross occlude reid beside close hug sign wall`) |
| `python3 tools/test_marker_follow.py all` | 25/25 | Marker following scenes (`follow`, `switch`) |
| `python3 tools/test_marker_pipeline.py` | 15/15 | Camera image → detector → marker tracker, and the page's JPEG. `--people` also loads the person detector (16/16; T4 must be down). |
| `python3 tools/test_ui_server.py` | 22/22 | Web page server: video first + boxes, full / LITE, selection, deadman per page |
| `python3 tools/test_follow_lidar.py main` / `wall` | 4/4, 3/3 | Old automatic follow mode |
| `python3 tools/test_odom_zupt.py [bag]` | — | Odometry zero-velocity update on a recorded stop |
| `python3 tools/test_ground_seg.py [bag]` · `test_ground_scan.py [bag]` | — | Ground segmentation on a recorded cloud |

`SCAN_RANGE_MIN=0.3` before `test_click_follow.py` / `test_marker_follow.py` / `test_follow_controller.py` gives their simulated scan the
near-field range (`near_field:=true`); the default 1.0 is p2l_safety's.

## 12. FCU parameters

Reading is safe. Writing changes how the vehicle behaves: note the old value first.

```bash
ros2 param get /mavros/param WP_SPEED          # read
ros2 param set /mavros/param WP_SPEED 2.0      # write (operator's decision)
```

`WP_SPEED` caps the speed in GUIDED and must be at least the follow `max_linear` (0.6).

## 13. Git

```bash
git status                      # what changed
git log --oneline -10           # recent commits
git add -A && git commit -m "…" # commit
git push origin main            # publish to github.com/AbhinavGarg815/dagger_ros2_ws (private)
bash tools/sync_learnings.sh    # copy Claude's project memory into docs/learnings/ (before a wrap-up commit)
```

In the repository: code, `COMMANDS.md`, the reports (`docs/reports/`) and the learnings (`docs/learnings/`).
Not in it: bags (`~/dagger_bags/`), models (`models/`), third-party sources (`src/<vendored>/`).

## 14. Quick reference

| Item | Value |
|---|---|
| Workspace | `/home/jetson/ros2_ws` |
| Follow page | `http://<rover-ip>:8080` |
| Operator-app gateway | `ws://<rover-ip>:8090/api` (8081 round-trip test, 8082 fake rover on the PC) |
| Foxglove | `ws://<rover-ip>:8765` |
| SIYI link (Jetson port `enP8p1s0`) | Jetson `192.168.144.100` · air unit `.11` · ground unit `.12` · MK32 Android `.20` |
| Bags | `~/dagger_bags/` (replays in `~/dagger_bags/replay/`) |
| Saved maps | `src/dagger_bringup/maps/` |
| Parameters | `src/dagger_bringup/params/` |
| Node logs | `~/.ros/log/` |
| Issues and fixes report | `docs/reports/issues_and_fixes.md` (`/home/jetson/dagger_reports/` links here) |
| Learnings (Claude's project memory, copy) | `docs/learnings/claude-memory/` |
