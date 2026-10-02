# Dagger UGV — command reference

Every command used to build, run, test and debug the rover, in the order you need them.
Last updated: 2026-10-02 (reports + learnings in the repo).

**Conventions**

- Run everything from the workspace, in a terminal that has the environment loaded:
  ```bash
  cd /home/jetson/ros2_ws
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ```
- Tags: **[MOTION]** the rover may move — operator runs it, RC transmitter in hand ·
  **[SUDO]** needs root · everything else only reads or configures.
- `T1`…`T5` are the terminals of the running stack (section 3). Start them in that order.
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

Engines in `models/` are tied to this GPU and TensorRT version: rebuild them after a JetPack update.

## 2. Build

| Command | Use |
|---|---|
| `colcon build --packages-select dagger_bringup` | Rebuild our nodes, launch files, params, web page (seconds). |
| `colcon build --packages-select dagger_msgs dagger_bringup` | After changing a message definition (about 1 min). |
| `colcon build --packages-select dagger_description` | After changing the URDF or its launch files. |

A running node keeps its old code: restart the terminal that runs it after a build.
The install is not symlinked — an edit in `src/` does nothing until built.

## 3. Bring-up

| Terminal | Command | What it starts |
|---|---|---|
| T1 | `ros2 launch dagger_bringup drive_bringup.launch.py` | FCU link (mavros), LiDAR, robot model. Add `use_camera:=true` for following. |
| T2 | `ros2 launch dagger_bringup nav2_gps.launch.py` | Odometry, GPS map frame, Nav2, costmaps, collision monitor, command path to the FCU. Add `map_source:=slam` when T5 localizes. |
| T3 | `ros2 launch dagger_bringup foxglove.launch.py` | Foxglove bridge on `ws://<rover-ip>:8765`. Add `with_cloud:=true` for the raw point cloud (not while driving). |
| T4 | `ros2 launch dagger_bringup follow.launch.py …` | Person / marker following and its web page (section 7). |
| T5 | `ros2 launch dagger_bringup slam.launch.py …` | 2D SLAM (section 6). |

Then, **before any drive and again after every T1 or mavros restart**:

```bash
python3 tools/preflight.py            # configure the FCU link + health check: must end in PASS
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
| `bash tools/estop_clear.sh` | Release the E-stop. The mode stays HOLD: re-select GUIDED yourself. |
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
| Foxglove 3D panel → publish **Pose** on `/goal_pose_ui` | **[MOTION]** Click-to-goal. **Pose estimate** on `/initialpose_ui` re-seeds SLAM localization. |
| `bash tools/solo_goal.sh <bag> "8" 15:20:00 15:22:00` | **[MOTION]** Solo test: at each clock slot, if the safety gate passes, records and sends the goal. |
| `python3 tools/solo_gate.py --save` | Remember the start pose for the solo gate (`solo_gate.py` alone = check it). |
| `bash tools/pf_tune.sh show` | Current path-following parameters. `tuned` / `base` apply a profile live (lost at T2 restart). |

## 6. SLAM

**Mapping** (runs beside everything, publishes nothing into the real TF tree):

```bash
ros2 launch dagger_bringup slam.launch.py                     # T5, shadow mapping
bash tools/slam_record.sh <name>                              # record the drive (add `clouds` for raw point clouds)
# save, while T5 is still running:
ros2 run nav2_map_server map_saver_cli -t /slam/map -f <path> --ros-args -p map_subscribe_transient_local:=true
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '<path>'}"
# tie the map to GPS (needed for lat/lon goals and the automatic start pose):
python3 tools/slam_georef.py <bag> <poses.csv> src/dagger_bringup/maps/<name>_geo.yaml
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
| `python3 tools/follow_gate_test.py` | Bench test of the follow collision monitor. **Disarmed only** — it refuses otherwise. |
| `ros2 launch dagger_bringup follow.launch.py target_source:=selector` | Old automatic mode: follows whoever is ahead, no page. |
| `bash tools/follow_solo.sh <bag> 14:05:00 14:07:00` | **[MOTION]** Solo test of the automatic mode: enables follow at a clock slot if the gate passes. |

## 8. Recording

| Command | Records | Read with |
|---|---|---|
| `bash tools/pf_record.sh <name>` | Path following and follow commands, odometry, FCU I/O, TF, logs. No point clouds. | `pf_analyze.py`, `stop_distance.py` |
| `bash tools/track_record.sh <name>` | Everything the people / marker trackers see and output, the page's video, follow commands. | `replay_tracker.py`, `compare_detectors.py` |
| `bash tools/slam_record.sh <name> [clouds]` | Scans, odometry, GPS, TF for re-mapping. | `slam_replay.sh` |
| `bash record_drive.sh` | Full drive including raw point clouds (large). | `test_ground_seg.py` |

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
| Is a sensor or topic alive, at what rate? | `ros2 topic hz /rslidar_points` · `/scan` · `/scan_safety` · `/odometry/filtered` · `/d435/color/image_raw` |
| What is following doing? | `ros2 topic echo /follow/state` · `python3 tools/follow_status.py` |
| Who publishes / subscribes a topic? | `ros2 topic info -v <topic>` |
| Is a lifecycle node active? | `ros2 lifecycle get /collision_monitor_follow` (or `/controller_server`, `/planner_server`, …) |
| Is a transform there, who owns it? | `ros2 run tf2_ros tf2_echo map odom` |
| A node's parameter | `ros2 param get <node> <name>` · `ros2 param list <node>` |
| Obstacles in the costmap that are not there | `python3 tools/costmap_audit.py [local\|global]` |
| Does the costmap see a low object ahead? | `python3 tools/scan_ahead.py` · `python3 tools/low_obstacle_approach.py` |
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
| `python3 tools/test_follow_controller.py` | 26/26 | Follow controller: speeds, search, close-range hold, CLEAR |
| `python3 tools/test_click_follow.py all` | 53/53 | Person following scenes (or one of `main cross occlude reid beside close hug sign wall`) |
| `python3 tools/test_marker_follow.py all` | 25/25 | Marker following scenes (`follow`, `switch`) |
| `python3 tools/test_marker_pipeline.py` | 15/15 | Camera image → detector → marker tracker, and the page's JPEG. `--people` also loads the person detector (16/16; T4 must be down). |
| `python3 tools/test_ui_server.py` | 22/22 | Web page server: video first + boxes, full / LITE, selection, deadman per page |
| `python3 tools/test_follow_lidar.py main` / `wall` | 4/4, 3/3 | Old automatic follow mode |
| `python3 tools/test_odom_zupt.py [bag]` | — | Odometry zero-velocity update on a recorded stop |
| `python3 tools/test_ground_seg.py [bag]` · `test_ground_scan.py [bag]` | — | Ground segmentation on a recorded cloud |

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
| Foxglove | `ws://<rover-ip>:8765` |
| SIYI link (Jetson port `enP8p1s0`) | Jetson `192.168.144.100` · air unit `.11` · ground unit `.12` · MK32 Android `.20` |
| Bags | `~/dagger_bags/` (replays in `~/dagger_bags/replay/`) |
| Saved maps | `src/dagger_bringup/maps/` |
| Parameters | `src/dagger_bringup/params/` |
| Node logs | `~/.ros/log/` |
| Issues and fixes report | `docs/reports/issues_and_fixes.md` (`/home/jetson/dagger_reports/` links here) |
| Learnings (Claude's project memory, copy) | `docs/learnings/claude-memory/` |
