# Dagger UGV — session hand-off prompt (written 2026-10-02)

Paste everything below the line into a new session.

---

Continue the Dagger UGV project: a 350 kg tracked rover, outdoor only. ROS 2 Humble on a Jetson Orin
Nano 8 GB, ArduPilot Rover on a Cube Orange+ over MAVROS, RS-Helios-16P LiDAR, RealSense D435
(colour only, mounted upside down), dual-antenna GNSS. Workspace `/home/jetson/ros2_ws`
(git, private remote github.com/AbhinavGarg815/dagger_ros2_ws, in sync at `15206ff`).

**This session's goal: close the obstacle-avoidance gaps.** Plan first, get my OK, then build.

## Read first, in this order

1. `/home/jetson/ros2_ws/COMMANDS.md` — every command (bring-up, stop, tests, debugging).
2. `/home/jetson/ros2_ws/docs/reports/issues_and_fixes.md` (the old `/home/jetson/dagger_reports/`
   paths are symlinks to `docs/reports/`; edit the repo files), these entries in full:
   N7 + N8 (collision monitor, braking) · Q29–Q32 (ground segmentation, low obstacles erased,
   low-obstacle memory shelved) · Q58 (near-field blind zone found) · "Open items" (P13, P14).
3. Your memory index (`MEMORY.md`) and the files it points to.
4. `git log --oneline -15` and `git status` (an interrupted session once left uncommitted edits —
   review any diff you find before building on it).

## Standing rules (from me, still in force)

- **I drive all motion.** Never send goals, arm, change modes or publish velocity. Give me the exact
  command and say what the rover will do (straight, turn left, stop at …) before each test.
- **FCU parameters are mine.** Read with a single GetParameters call; for writes give me
  `ros2 param set /mavros/param <NAME> <value>` with the old value to undo.
- **Safety configuration changes need my OK before you apply them** (collision monitor zones,
  `/scan_safety`, anything the stop path depends on).
- I start the terminals (T1 `drive_bringup`, T2 `nav2_gps`, T3 `foxglove`, T4 `follow`, T5 `slam`).
  Before any drive `python3 tools/preflight.py` must PASS. RC transmitter in my hand = E-stop.
- While I run a timed test, don't poll or block — wait for "done", then read the bag. Stop the
  recorder before reading a bag.
- Update the issues report and `COMMANDS.md` **in the same session** as each fix, finding or new
  command. Commit as Abhinav with **no co-author trailer**. At each session wrap-up: run
  `bash tools/sync_learnings.sh`, commit the reports + learnings, and push — I want GitHub to hold
  the code, reports and learnings, kept current. Ask before pushing at other times.
- Kill by PID, never `pkill -f <pattern>` (it has matched the calling shell).
- I want a plan before implementation; recommend one option rather than listing all of them.

## Where the project stands (my estimate, 2026-10-02)

| Capability | State |
|---|---|
| Object following ~80 % | Person (click-to-select in crowds, LiDAR + appearance re-ID, search/resume) and ArUco marker following DRIVEN live; browser page also on the SIYI MK32 handset (~100 ms video). |
| SLAM ~70 % | 2D slam_toolbox maps of the test lanes; localization owns map->odom, seeded by GPS + scan match; Nav2 and lat/lon goals in the map work live (0.23 / 0.38 m). |
| Waypoint following ~70 % | Nav2 path following tuned; relative, lat/lon, FCU-mission and Foxglove-click goals work live. |
| **Obstacle avoidance ~55 %** | Emergency layer solid; low and near-field obstacles are the gaps (below). |
| Road route following 0 % | Not started. |

## The task: obstacle-avoidance gaps, in this order

**What already works (don't rebuild):** `nav2_collision_monitor` on the Nav2 path (stop 0.6 m ahead of
the bumper ±0.70, slow ×0.4 to 1.75 m ±0.85, footprint approach 1.5 s) and a second instance on the
follow path, both fed by `/scan_safety`; live: stops 0.56 m from a person, rolls 0.01 m.
`cmd_vel_safety` zeroes all output if `/scan_safety` is > 0.5 s stale (Humble treats a stale source
as "no obstacles"). Costmaps: `/scan` from Patchwork++ ground segmentation (floor 0.12 m, 17 % of a
core) plus the old band `/scan_band` (0.35–0.75 m); `expected_update_rate` 1.0 s on both, so a dead
scan fails closed. Theta* planner replans around costmap obstacles.

### 1. Near-field blind zone (P13) — safety, needs my OK before applying

`/scan_safety` (pointcloud_to_laserscan `p2l_safety` in `nav2.launch.py`, frame base_footprint,
band 0.30–1.60 m, angle increment 0.0087) has **range_min 1.0 m** to drop the mast/mount
self-returns (r ≤ 0.96 m). The footprint is ±0.85 × ±0.58 m, so the corners sweep a 1.03 m circle
when the rover turns in place: the first 0.15 m in front of the bumper (inside the STOP polygon,
which starts at x = 0.85) and the whole corner-sweep ring are invisible to BOTH collision monitors.
Follow-me has a controller-side mitigation only (no turning in place within 1.6 m of the target);
Nav2 in-place rotations and anyone who isn't the target are unprotected.
- Proposed: I record the raw cloud parked with nobody near (`/rslidar_points` + `/tf_static`, a few
  seconds; no motion); find the self-returns in base_footprint; replace range_min 1.0 with a body
  filter around hull + mast and range_min ~0.3. `src/dagger_bringup/dagger_bringup/self_filter.py`
  (box filter, currently UNUSED; box x −0.90…0.45, y ±0.60, z −0.30…1.30) is a starting point —
  verify it against the data. Patchwork++ also uses min_range 1.0 for the mast.
- **Everything else reading `/scan_safety` changes with it** — check each and re-run its tests:
  both collision monitors, `scan_safety_watchdog`, follow_controller's forward slow zone
  ([0.85, 2.6] × ±0.85) and its `min_turn_range` comment ("blind inside 1.0 m"), person_tracker
  (`min_range` 1.0, `near_range` 1.0 — the "too close for the LiDAR" target exists because of this
  blind zone), marker_tracker (`min_range` 0.3).
- Validate: static walk test like N7 (no false triggers over ~10 min parked; a person at the
  corner while Nav2 turns in place → stop), then the offline suites in COMMANDS.md §11.

### 2. Low obstacles forgotten inside ~3.3 m (P14, my "task 2")

The LiDAR is 1.12 m up with its lowest beam at −15°: it meets flat ground ~3.9–4.5 m out, and a
20 cm box is cleanly visible only in windows (~3.7–4.0, 4.3–4.6, 5.0–5.4 m from the rover centre).
Measured live (Q30): box marked at 4.36 m, lost at ~3.5 m, **ERASED at 3.23 m** by empty rays.
- Tried and shelved (Q30–Q32, code kept, `low_obstacle_memory:=true`): a separate
  `low_obstacle_layer` fed by `/scan_low` (low-only returns), ray-clearing only beyond 4 m. Failed
  because **a person walking past left a blob that stayed**. Ideas recorded then: widen the
  "tall return above it" check to ±2 rays, and require a low return to persist N scans at the same
  odom cell before it enters memory (people move, boxes don't). Also consider clearing a remembered
  cell only when the beam geometry says that cell is actually observable.
- Related gap (Q32): negative obstacles — a clean drop-off with nothing at its edge is not marked.
  Note it; don't solve it here unless cheap.

### 3. A dedicated live detour test

No deliberate "drive around a placed obstacle" test is on record. Tall box first, then a low box.
**Compute the beam geometry before telling me where to put a low object** (Q30: the first box was
placed inside the blind band). Record with `bash tools/pf_record.sh <name>`; analyse with
`pf_analyze.py`; `tools/low_obstacle_approach.py`, `scan_ahead.py` and `costmap_audit.py` exist.

### 4. Later: obstacle avoidance on the follow path

Following is stop-only today (collision monitor + forward slow zone); it never plans around.

## Lessons that cost time before

- Humble's collision monitor: an approach polygon whose footprint topic never arrives fails OPEN,
  silently; a stale observation source counts as "no obstacles".
- rclpy: one source line logging at two severities crashes the node ("Logger severity cannot be
  changed between calls"). Assigning `bytes` to a uint8[] field costs ~30 ms per 160 KB — use
  `array.array('B', …)`. A Python TF listener costs 16–25 % CPU — use /tf_static + odometry.
- Test in an isolated ROS domain (the existing tests use 42 and 77). Run timing-sensitive scene tests
  with the live stack down. With the stack up only ~1.2 GB RAM is free — don't load a second GPU
  model beside it.
- Verify with data before concluding: e.g. an "odometry rolls backward" signal turned out to be
  estimator slide when checked against a LiDAR scan match; a "SEARCHING" episode turned out to be
  the marker cut by the image edge. Define analysis windows from the physics of the test.
- A stale ros2 daemon reports 0 publishers: `ros2 daemon stop; ros2 daemon start`.
- Foxglove `with_cloud:=true` eats a core; leave it off while driving.

## Also open, not this session (details in the report's Open items)

Crowd test with 2–3 helpers (P12) · LiDAR hold for a marker's carrier · re-test CLEAR live ·
handset latency/CPU options (P17) · GPS waypoints in open sky vs Mission Planner AUTO (P6) ·
track imbalance SERVO1_MAX (P5) · FS_THR_ENABLE (P4) · odometry slide after stops (P10/P16) ·
road route following.

Start by reading, then give me the plan for gap 1 with the exact data-capture command for me to run.
