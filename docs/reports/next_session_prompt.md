# Dagger UGV — session hand-off prompt (written 2026-10-03 wrap-up, for the next session)

Paste everything below the line into a new session.

---

Continue the Dagger UGV project: a 350 kg tracked rover that must work **outdoors and indoors**.
ROS 2 Humble on a Jetson Orin Nano 8 GB, ArduPilot Rover on a Cube Orange+ over MAVROS,
RS-Helios-16P LiDAR, RealSense D435 (colour only), dual-antenna GNSS. Workspace
`/home/jetson/ros2_ws` (git, private remote github.com/AbhinavGarg815/dagger_ros2_ws, pushed at the
2026-10-03 wrap-up).

**This session's goal: finish the LiDAR odometry (P20, report Q72-Q73).** Plan first, get my OK, then build.

## Read first, in this order
1. `docs/reports/issues_and_fixes.md`: Q72 (odometry stalled under degraded GNSS), Q73 (LiDAR-odometry
   research + benchmark + the C++ node and its open bug), Q71 (band top 1.35 m, overhang layer), Open items.
2. `src/dagger_perception/src/lidar_odom.cpp` (header says what is wrong), `tools/test_lidar_odom.py`,
   `tools/lo_bench.py`, `tools/lo_eval.py`.
3. `COMMANDS.md` sections 2 (terminals) and 11 (tests).

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


## Where it stands (2026-10-03)
- Obstacle avoidance: near field (P13), low-obstacle memory (P14), detour clearance, Foxglove goals in
  odom, turn deadlock (Q70), overhangs (Q71: safety band top 1.60 -> 1.35 m = rover top 1.17 + margin;
  overhang costmap layer) — all built, defaults on, live-checked. Remaining: drop-offs (P18),
  follow-path avoidance, forward-only stop zone (a stop-zone hit blocks in-place turns; Q72 goal 3).
- **P20 (Q72):** `/odometry/filtered` takes its velocity from the FCU's EK3 (GNSS velocity). Near the
  wall / under the lane-end canopy (h_acc 3-4 m, dual-antenna heading invalid) it read stopped/backing
  while the rover drove 1.1 m — costmap smeared, goal refused; the FCU also overspeeds there.
- **Q73 selection (done):** small_gicp VGICP scan-to-model, 0.5 m Gaussian voxels (LRU 100/10), 0.2 m
  downsampling, 10-neighbour covariances, constant-velocity guess, no deskew. Benchmark over 6 drives /
  13 k scans beat KISS-ICP (pivot heading errors, jitter) and GenZ-ICP (CPU spikes > 1 s), ~35 % of a core.
  Python reference (pip small_gicp) is stable on every drive.
- **C++ `lidar_odom` (built, NOT launched):** fox3 lockstep replay matches the Python pipeline scan for
  scan for ~35 parked scans, then the constant-velocity guess creeps (-0.003 -> -0.014 -> -0.02 m/scan)
  and it diverges. Checked and ruled out: Eigen angle maths, the covariance setter, optimizer state
  between calls, -march flags, float vs double transform (Python with float is stable). `debug:=true`
  logs map size / guess / result / error per scan; the Python side is in lo_bench.py's vgicp05.

## Next steps (my plan, for your OK)
1. Find the C++/Python difference (first suspect: the step/guess bookkeeping or the voxel map built by
   the node vs the pip build — e.g. build the node against the pip wheel's headers, or run the Python
   pipeline through the node's exact inputs) until `tools/test_lidar_odom.py` passes on fox3, then on
   all 6 drives with the same accuracy as lo_eval's vgicp05.
2. Calibrate the covariance (cov_scale, degenerate_info: corridor 6.7 k vs real 150-180 k).
3. Live, disarmed: run `lidar_odom` beside the stack, compare with `/odometry/filtered`, CPU by PID.
4. EKF: fuse its LINEAR velocity (yaw stays gyro + dual-antenna heading), behind a launch arg; then
   the guard (LiDAR vs FCU velocity disagreement -> zero Nav2 commands; stop path — needs my OK).
5. Record an indoor corridor drive (RC) for validation. Indoors the FCU has no position/yaw source:
   ExternalNav (EK3 source set 2) will be needed — FCU parameters are mine.

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
P18 drop-offs · forward-only stop zone · follow-path avoidance · turn-into-box stop margin at 0.6 rad/s ·
Nav2 task 2 · handset latency/CPU options.
