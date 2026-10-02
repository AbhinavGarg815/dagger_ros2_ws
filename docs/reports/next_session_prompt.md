# Dagger UGV — session hand-off prompt (written 2026-10-02 wrap-up, for 2026-10-03)

Paste everything below the line into a new session.

---

Continue the Dagger UGV project: a 350 kg tracked rover, outdoor only. ROS 2 Humble on a Jetson Orin
Nano 8 GB, ArduPilot Rover on a Cube Orange+ over MAVROS, RS-Helios-16P LiDAR, RealSense D435
(colour only, mounted upside down), dual-antenna GNSS. Workspace `/home/jetson/ros2_ws`
(git, private remote github.com/AbhinavGarg815/dagger_ros2_ws, pushed at the 2026-10-02 wrap-up).

**This session's goal: obstacle-avoidance gap 2 — low obstacles forgotten inside ~3.3 m (P14).**
Gap 1 (near field) was closed yesterday. Plan first, get my OK, then build.

## Read first, in this order

1. `/home/jetson/ros2_ws/COMMANDS.md` — every command (bring-up, stop, tests, debugging).
2. `/home/jetson/ros2_ws/docs/reports/issues_and_fixes.md`, these entries in full: **Q65** (near field,
   done yesterday — safety_scan, bench + armed results, limits) · Q29–Q32 (ground segmentation, low
   obstacles erased, low-obstacle memory shelved) · section **"Next session plan — obstacle avoidance
   day 2"** · "Open items" (P13, P14, **P18 drop-offs**).
3. Your memory index (`MEMORY.md`) and the files it points to — especially the obstacle-avoidance
   project file and the feedback on drop-offs.
4. `git log --oneline -15` and `git status` (review any diff you find before building on it).

## Standing rules (from me, still in force)

- **I drive all motion.** Never send goals, arm, change modes or publish velocity. Give me the exact
  command and say what the rover will do (straight, turn left, stop at …) before each test.
- **FCU parameters are mine.** Read with a single GetParameters call; for writes give me
  `ros2 param set /mavros/param <NAME> <value>` with the old value to undo.
- **Safety configuration changes need my OK before you apply them** (collision monitor zones,
  `/scan_safety`, anything the stop path depends on).
- I start the terminals (T1 `drive_bringup`, T2 `nav2_gps`, T3 `foxglove`, T4 `follow`, T5 `slam`).
  Before any drive `python3 tools/preflight.py` must PASS. RC transmitter in my hand = E-stop.
- **Start T2 with `near_field:=true`** (default is still false until a normal drive shows no false
  stops — then flip the default in `nav2.launch.py` and `nav2_gps.launch.py`, with my OK).
- **Never pick a test line, heading or goal from the costmap alone** — it cannot see drop-offs.
  Ask me what bounds the site. The test lane has a wall on the right and **steps (a drop-off) along
  the whole left side** (the "obstacles" on the left are plants).
- While I run a timed test, don't poll or block — wait for "done", then read the bag. Stop the
  recorder before reading a bag.
- Update the issues report and `COMMANDS.md` **in the same session** as each fix, finding or new
  command. Commit as Abhinav with **no co-author trailer**. At each session wrap-up: run
  `bash tools/sync_learnings.sh`, commit the reports + learnings, and push. Ask before pushing at
  other times.
- Kill by PID, never `pkill -f <pattern>` — and list before killing: a `pgrep -f` pattern also
  matches the shell that runs it.
- I want a plan before implementation; recommend one option rather than listing all of them.

## Where the project stands

| Capability | State |
|---|---|
| Object following ~80 % | Person + ArUco following driven live; page on the SIYI MK32 handset. |
| SLAM ~70 % | 2D maps of the test lanes; localization + Nav2 goals in the map work live. |
| Waypoint following ~70 % | Nav2 path following tuned; relative, lat/lon, mission and click goals live. |
| **Obstacle avoidance ~65 %** (my estimate after 10-02) | Emergency layer + **near field closed** (Q65). Gaps: low obstacles inside ~3.3 m (P14), drop-offs (P18), no detour test on record, follow path is stop-only. |
| Road route following 0 % | Not started. |

## Done yesterday (gap 1, Q65) — don't rebuild

`safety_scan` (C++, new package `dagger_perception`) replaces p2l_safety with T2 `near_field:=true`:
body box x −0.90…0.50, y ±0.60 (the rover's own frame and GNSS posts, measured from captures) +
range_min 0.3; bit-identical to p2l with the box off; 6.3 vs 13.0 % CPU. PolygonStop starts at x 0.50
in both monitors. Live: 10 min parked no false trigger; operator beside the hull → turn cut / STOP;
**armed Nav2 turn into a box stopped after 22° with 13 cm to spare**, no coasting. Limits: a zone needs
> 3 rays (things < ~3 cm wide at 1 m ignored); mixed-pixel phantom points off the rails' edges when an
object is just beyond them (conservative); low objects / children beside the hull invisible.

## Today: gap 2 (P14), the plan I approved in outline

1. **Approach-and-stop baseline in the lane (no detour — the lane is too narrow and the step side is
   invisible to Nav2):** cardboard box 20–25 cm tall, 40–60 cm wide, 8 m ahead of the rover's centre
   at the rover's distance from the wall (~2.2 m); recorder `bash tools/detour_record.sh approach_low`;
   goal 6 m along the lane, parallel to the wall (`send_goal.py 6 --left <correction>` — re-measure
   the rover's heading against the wall first); the rover stops ~2 m before the box. Confirm with me
   that the step edge is ≥ 1 m from the rover's left side. Purpose: see when the box is erased from
   both costmaps, and get real clouds to replay.
2. **Build low-obstacle memory v2:** C++ node `low_scan` in dagger_perception on
   `/patchworkpp/nonground` (same /tf_static + body box approach as safety_scan): low-only scan in one
   pass (12–35 cm returns, dropped where something 40–160 cm is within ±2 bearings) + a **persistence
   gate** (a low return counts only after ~0.5 s in the same ~15 cm odom cell — people move, boxes
   don't; odometry from /odometry/filtered, no TF listener) → `/scan_low` → the existing
   `low_obstacle_layer` (clears only beyond 4 m), replacing p2l_low_all + p2l_tall + low_scan_filter.
   Behind `low_obstacle_memory:=true`, default off until validated.
3. **Offline tests:** synthetic; replay the 10-02 walk-around capture (`body_walk_132959`, the
   operator circling the rover → must leave NO mark); replay the approach bag (the box must stay in the
   costmap down to the stop).
4. **Live:** someone walks past at 2–5 m (no blob); approach-and-stop again with memory on.
5. **Later:** the real detour test (gap 3) needs a site with room on both sides or the step side
   blocked with tall markers; drop-off detection (P18); avoidance on the follow path.

## Lessons that cost time before

- Humble's collision monitor: an approach polygon whose footprint topic never arrives fails OPEN;
  a stale source counts as "no obstacles"; after a stop it publishes zeros for `stop_pub_timeout`
  (2 s) and then goes silent (cmd_vel_safety then holds the FCU setpoint at 0). Its approach scaling
  is one 0.1 s step more conservative than a straight model; PolygonSlow scales turns too.
- To reproduce pointcloud_to_laserscan exactly, compute range/bearing in **float** (its unqualified
  `hypot`/`atan2` on floats pick the float overloads) — in double 93 of 1.1 M bins differed.
- A Python node only *receiving* the 0.8 MB / 10 Hz cloud costs ~10 % of a core — cloud work goes
  in C++. rclpy: one source line at two log severities crashes the node; assign `array.array('B', …)`
  to uint8[] fields, never bytes; a Python TF listener costs 16–25 % — use /tf_static + odometry.
- A thin "object" the operator cannot see may be a LiDAR **mixed pixel** (one ring, one azimuth
  window, ranges between a near edge and the object behind it).
- Test in an isolated ROS domain (42, 77, 78 used). Run timing-sensitive scene tests with the live
  stack down. With the stack up only ~1.2 GB RAM is free.
- Verify with data before concluding; define analysis windows from the physics of the test.
- A stale ros2 daemon reports 0 publishers: `ros2 daemon stop; ros2 daemon start`.
- Foxglove `with_cloud:=true` eats a core; leave it off while driving.

## Also open (details in the report's Open items)

Crowd test (P12) · LiDAR hold for a marker's carrier · re-test CLEAR live · handset latency/CPU
(P17) · GPS waypoints in open sky (P6) · SERVO1_MAX (P5) · FS_THR_ENABLE (P4) · odometry slide
(P10/P16) · road route following.

Start by reading, then confirm the site with me and give me the approach-and-stop test with the
exact commands.
