# Dagger UGV — session hand-off prompt (written 2026-10-06 wrap-up, for the next session)

Paste everything below the line into a new session.

---

Continue the Dagger UGV project: a 350 kg tracked rover that must work **outdoors and indoors**.
ROS 2 Humble on a Jetson Orin Nano 8 GB, ArduPilot Rover on a Cube Orange+ over MAVROS,
RS-Helios-16P LiDAR, RealSense D435 (colour only), dual-antenna GNSS. Workspace
`/home/jetson/ros2_ws` (git, private remote github.com/AbhinavGarg815/dagger_ros2_ws, pushed at the
2026-10-06 wrap-up).

**Next session: first live localization drive on the indoor + outdoor map `mixed1`, then pick from the open
items with me.** Plan first, get my OK, then build.

## Read first, in this order
1. `docs/reports/issues_and_fixes.md`: Q80-Q82 (indoor GUIDED on ExternalNav, Foxglove waypoint routes, the `mixed1`
   map), Q79 (FCU velocity weight), then Open items.
2. `COMMANDS.md` sections 2 (terminals: T2 defaults, indoor rules), waypoints and SLAM.

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



## Where it stands (2026-10-06)
- **LiDAR odometry is fused by default** (T2 `lidar_odom:=true`; Q72-Q79). Q79 (FCU velocity σ 0.3 with LiDAR fusion)
  is applied but its distance scale is **not yet confirmed on a measured drive**.
- **Indoor GUIDED works** (Q80): `extnav_relay` feeds lidar_odom to the Cube as ExternalNav; EK3 source set 2 on
  RC10 (low = GPS, middle = indoor). **Flip RC10 only parked and disarmed** (each switch jumps the Cube's position
  3.5-4.7 m and heading ~44°; harmless for body-frame GUIDED). Indoors run `python3 tools/preflight.py --indoor`.
  Open (my FCU call): "PreArm: GPS and AHRS differ" can block re-arming indoors — `ARMING_CHECK 29816 → 29808` or not.
- **Foxglove waypoint routes** (Q81, live OK): pose tool on `/waypoint_ui`, Publish `/waypoints/go` / `/waypoints/clear`.
- **Map `mixed1`** (Q82): one continuous drive outdoor → door → indoor hall → back out, 161 m, loop closed; saved,
  georeferenced (GNSS here only 3-7 m → lat/lon goals ~3-5 m; odom/map clicks unaffected). The doubled line on the
  long wall is a real stepped structure (low face 0.25 m in front), not an error. Offline self-replay: localized,
  score median 0.04 m, driving corrections median 0.05 / max 0.99 m.
- Obstacle avoidance (Q65-Q71): near field (`near_field:=true`, default still false), low-obstacle memory
  (`low_obstacle_memory:=true`, default false), overhang layer, safety band top 1.35 m.

## Next session, in order
1. **Live localization on `mixed1`**, start OUTDOORS (slam_geo_init searches ±8 m around GNSS): T1, T2
   `map_source:=slam`, T5 `slam.launch.py mode:=localization map:=mixed1`, preflight, `bash tools/slam_record.sh loc_mixed1`.
   Parked check (`seeded`, `localized`), then an RC loop through the door and back; then map-frame goals / a waypoint
   route across the door (indoors: RC10 middle, parked + disarmed).
2. In the same drive, a known straight distance (tape) to confirm Q79's distance scale.
3. Then pick with me: P18 drop-off detection (recommended next feature), flip `near_field` / `low_obstacle_memory`
   defaults after clean drives, P19 overhang live check.

## Open (pick with me)
- P18 drop-offs invisible to Nav2 · forward-only stop zone (a stop-zone hit blocks in-place turns) · follow-path
  avoidance · turn-into-box stop margin at 0.6 rad/s · P15 ArUco: LiDAR hold of the carrier, CLEAR re-test ·
  P12 crowd test + glass-to-glass latency · P6 GPS goals in open sky · P17 handset latency/CPU.
- FCU (mine): PreArm GPS-AHRS (`ARMING_CHECK`), P4 radio failsafe, P5 track imbalance (`SERVO1_MAX`).

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
- `map_saver_cli` on a big map needs `-p save_map_timeout:=20.0` (the 2 s default fails "spin map subscription").
- A 2D scan band takes the nearest return per bearing: a stepped structure (low plinth in front of a wall) draws as
  two parallel lines from different distances. Check single 3D sweeps before calling it a pose error.
- `ros2 launch` children survive killing the `ros2 run`/launch wrapper — check `ps` and kill the node PIDs.
