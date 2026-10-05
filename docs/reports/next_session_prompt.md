# Dagger UGV — session hand-off prompt (written 2026-10-05 wrap-up, for the next session)

Paste everything below the line into a new session.

---

Continue the Dagger UGV project: a 350 kg tracked rover that must work **outdoors and indoors**.
ROS 2 Humble on a Jetson Orin Nano 8 GB, ArduPilot Rover on a Cube Orange+ over MAVROS,
RS-Helios-16P LiDAR, RealSense D435 (colour only), dual-antenna GNSS. Workspace
`/home/jetson/ros2_ws` (git, private remote github.com/AbhinavGarg815/dagger_ros2_ws, pushed at the
2026-10-05 wrap-up).

**Next session: confirm the Q79 weighting live, then pick from the open items with me.** Plan first, get my OK, then build.

## Read first, in this order
1. `docs/reports/issues_and_fixes.md`: Q72-Q79 (LiDAR odometry: why, selection, the C++ bug, live shadow/fused runs,
   sticky goal checker, heading replay, twist-spike gate reverted, indoor tape test), then Open items.
2. `COMMANDS.md` sections 2 (terminals: T2 defaults) and 11 (tests).

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



## Where it stands (2026-10-05)
- **LiDAR odometry is live and the T2 default** (`lidar_odom:=true`): `lidar_odom` (dagger_perception, small_gicp
  VGICP, 0.5 m voxels) fused into `/odometry/filtered` as linear velocity; yaw stays gyro + dual-antenna heading.
  `lidar_odom:=shadow` runs it unfused, `false` reverts. ~45 % of one core. Validated: 7 drives offline = the Python
  reference; live shadow (outdoor + indoor), live fused (3/3, 5/5 goals); indoor tape: LiDAR 6.96/6.85 m vs ~7 m.
- **Q79 (applied, NOT yet driven):** with LiDAR fusion the FCU velocity is σ 0.3 m/s (was 0.1) — the fused
  odometry was 21-27 % short indoors; replay says −4/−6 % now. **First thing: one drive to confirm**
  (`bash tools/detour_record.sh <name>`, a few straight legs + turns; `tools/lo_bag_to_bench.py` + `lo_eval.py`).
- **Q76 StickyGoalChecker** (new pkg `dagger_nav_plugins`): goal xy tolerance 0.4 → 0.8 m once reached, so the
  final turn (tracks pivot ~0.07 m behind base_footprint) no longer hunts. Live 5/5. Known: after a big final
  turn the rover sits ~0.4-0.6 m from the clicked point.
- **Decided with data, no change:** heading set-up (Q77, offline EKF replay — keep gyro + GNSS heading);
  no stop-path speed guard (FCU overspeed p95 ≤ 0.10 m/s); LiDAR twist spikes at turn ends minor — a gate was tried
  and reverted (Q78). Future option only: lidar_odom as the primary odom (EKF fallback).
- Obstacle avoidance (Q65-Q71) done: near field, low-obstacle memory, overhang layer, band top 1.35 m.
- Tools: `tools/ekf_replay.py` (+ `ekf_replay_score.py`) replays ekf_local offline from bags recorded since Q75;
  `lo_bag_to_bench.py`, `lo_node_bench.py`, `lo_eval.py` score odometries without ground truth (venv with the
  small_gicp pip wheel; OMP/OPENBLAS threads = 1). Two replays at once collide (same domains) — run sequentially.

## Open (pick with me)
- Indoor GUIDED needs the FCU on ExternalNav (EK3 source set 2 from LiDAR odometry) — FCU parameters are mine.
  Also `FS_EKF_ACTION = 0` (EKF failsafe disabled) — my call.
- P18 drop-offs invisible to Nav2 · forward-only stop zone (a stop-zone hit blocks in-place turns) ·
  follow-path avoidance · turn-into-box stop margin at 0.6 rad/s · Nav2 task 2 · handset latency/CPU (P17).

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


- **Don't over-engineer:** check the live impact with data before building; small effect → record and move on;
  a fix that doesn't clearly win → revert, don't stack another layer (2026-10-05).
- When porting an odometry loop, never trust `Isometry3d::inverse()` on optimiser outputs — re-orthonormalise
  (Q73: the rotation drift doubled per scan through the constant-velocity guess).
- A new message field breaks decoding of recorded bags (Fast CDR is strict) — derive it from existing fields.
