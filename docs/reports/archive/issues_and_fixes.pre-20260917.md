# Dagger UGV — Issues & Fixes Log (running report)

Private running log of every problem hit on the Dagger project — symptom, root cause,
fix, and status. **Not** in the shared workspace. To be finalized at project end.

Owner: Abhinav · Location: `/home/jetson/dagger_reports/` (nvidia-only)
Last updated: 2026-07-13

**Status legend:** ✅ SOLVED · 🟡 WORKAROUND/PARTIAL · 🔴 OPEN · ⓘ insight/understood

---

## A. Build & Toolchain

**A1 — mrg_slam OOM during compile** ✅
- *Symptom:* build killed; system load ~15; swap exhausted.
- *Cause:* `MAKEFLAGS=-j6` × colcon package-parallelism = ~23 concurrent `cc1plus` on the 8 GB Orin.
- *Fix:* build mrg_slam single-threaded (`MAKEFLAGS=-j1 colcon build --packages-select mrg_slam --parallel-workers 1`).

**A2 — mrg_slam `--symlink-install` stale artifact** ✅
- *Symptom:* `failed to create symbolic link … python_scripts because existing path cannot be removed: Is a directory`.
- *Cause:* leftover non-symlink artifacts clashing with `--symlink-install`.
- *Fix:* full clean **no-symlink** rebuild (`rm -rf build install log; colcon build`).

**A3 — Incremental colcon build keeps STALE installed launch/param files** ✅ 🟡
- *Symptom:* edited a launch/yaml, rebuilt "successfully", but the installed copy was unchanged → old behavior at runtime.
- *Cause:* for this ament_python pkg, `data_files` weren't recopied on an incremental build.
- *Fix:* `rm -rf build/<pkg> install/<pkg>` then rebuild; **always grep the installed file** for the new markers after building.

---

## B. ROS / QoS / Middleware

**B1 — self_filter published BEST_EFFORT → mrg_slam received nothing** ✅
- *Symptom:* mrg_slam had no cloud input despite the topic publishing.
- *Cause:* QoS mismatch — mrg_slam prefilter subscribes RELIABLE; a best_effort publisher is incompatible.
- *Fix:* publish RELIABLE. (Node later bypassed for latency, see B3.)

**B2 — `/mavros/state` echoes empty** ✅
- *Symptom:* `ros2 topic echo /mavros/state` returns nothing.
- *Cause:* topic is BEST_EFFORT; default echo is RELIABLE → incompatible.
- *Fix:* `--qos-reliability best_effort --qos-durability volatile`.

**B3 — self_filter latency ~137 ms/cloud + 1.5 s queue buildup** 🟡
- *Symptom:* large delay in the odometry-critical path.
- *Cause:* rclpy (de)serialize of an 0.8 MB PointCloud2 per message (~137 ms) + depth-10 queue buildup.
- *Fix:* depth=1 cut latency to ~0.27 s, but 137 ms compute is inherent → **node bypassed**; a C++ `pcl_ros` CropBox is the future replacement.

---

## C. Localization & Odometry

**C1 — Nav2 costmap "Could not find connection between odom and base_footprint"** ✅
- *Symptom:* costmap TF failure, no control output.
- *Cause:* plain `odom` frame doesn't exist — mrg_slam namespaces everything to `dagger/odom`.
- *Fix:* pointed costmap `global_frame` / behavior `local_frame` to `dagger/odom` (later superseded by the plain-`odom` KISS+EKF decouple).

**C2 — mrg_slam scan-matching odom latency GROWS unbounded (0.2 s → 5.7 s)** ✅
- *Symptom:* closed-loop turn control breaks after a few minutes; restarting the SLAM terminal resets it.
- *Cause:* mrg_slam can't sustain 10 Hz on the CPU-bound Orin.
- *Fix:* **decoupled** Nav2's control odom onto **KISS-ICP + robot_localization EKF** (fast `odom→base_footprint`); leave mrg_slam for `map→odom` only.

**C3 — `/odometry/filtered` heading reads bistable (−176°/0°) with zero rate** 🔴
- *Symptom:* non-physical heading during an ACRO teleop test — value flips between −176° and 0° while yaw-rate stays 0.
- *Cause:* SUSPECTED duplicate EKF/odom publishers (diagnostic was interrupted, unconfirmed).
- *Next:* check publisher count on `/odometry/filtered` + EKF node count on resume.

---

## D. Nav2 / Path & Control

**D1 — Backward goal "no valid path"** ✅
- *Symptom:* planner fails for a goal behind the robot.
- *Cause:* stale, un-cleared obstacles in the rolling costmap (scan-latency non-clearing).
- *Fix:* clear costmaps.

**D2 — RotationShim hands off to RPP too early → pursuit wobble** ✅
- *Symptom:* weak/gappy ~0.3 rad/s rotation instead of a committed pivot.
- *Cause:* `rotate_to_heading_once` + early handoff at 45°.
- *Fix:* shim owns the turn — `angular_dist_threshold 0.30`, `once:false`.

**D3 — RPP rotation command stuck ~0.27 rad/s (accel deadlock)** ✅
- *Symptom:* config commanded 1.0 rad/s but actual command sat ~0.27.
- *Cause:* RPP rate-limits the command against the MEASURED velocity (`cmd = measured ± accel·dt`); the vehicle (then unable to turn at all) never rotated → measured ≈ 0 → command trapped in the dead zone.
- *Fix:* raised `max_angular_accel` (later reverted once the real drivetrain fix landed — see E5).

**D4 — Turn overshoot / hunt (after drivetrain fixed)** 🟡
- *Symptom:* 180° turn now COMPLETES but overshoots the target ~15° and swings a few times before settling.
- *Cause:* open-loop steering + steering **stiction** (bimodal rotation, E4) + control latency.
- *Fix so far:* `rotate_to_heading_angular_vel 0.5`, `max_angular_accel 2.0`, `yaw_goal_tolerance 0.4` → hunt reduced, turn completes. Full fix = closed yaw-rate loop (E4 / F6).

---

## E. FCU / ArduPilot / Drivetrain  ← the core saga

**E1 — ArduPilot GUIDED reverts to HOLD** ✅
- *Symptom:* `armed:true` but `mode:HOLD`; velocity setpoints ignored.
- *Cause:* the GUIDED velocity setpoint stream stopped / GCS-throttle failsafe.
- *Fix:* re-set GUIDED before each drive and keep streaming (moot after abandoning GUIDED).

**E2 — Steering oscillation on any turn (right-left-right)** ⓘ→✅
- *Symptom:* rover hunts on every turn.
- *Cause:* first blamed on `ATC_STR_RAT` PIDs; actually a symptom of E5 (GUIDED can't turn this vehicle).
- *Fix:* superseded by the RC-override architecture (E5).

**E3 — Steering SIGN inverted** ✅
- *Symptom:* commanded left → rover spins right.
- *Cause:* vehicle turn direction is opposite the ROS/REP-103 convention.
- *Fix:* `invert_yaw` (GUIDED era) → later `steer_reversed=true` in `cmd_vel_to_rc`.

**E4 — Stick-slip / bimodal rotation** 🔴
- *Symptom:* rotation rate is either ~0 (stalled) or ~0.9 rad/s (broken free) — no smooth slow rotation.
- *Cause:* heavy tracked vehicle static friction + open-loop steering (no rate feedback).
- *Next:* close a yaw-rate loop (ACRO `ATC_STR_RAT`, see F6).

**E5 — ★ ROOT CAUSE: ArduPilot GUIDED cannot turn this vehicle** ✅
- *Symptom:* turns produce ±full-scale steering slam (PWM 1185↔1819); backward drives forward; pure yaw is undefined/erratic.
- *Cause:* it's a **tracked steering+throttle-mixer (Ackermann-configured) rover**; GUIDED velocity control derives heading from the velocity VECTOR, which is undefined at zero speed and wrong even while arcing. **MANUAL + TX drives perfectly → the hardware/mixer are fine; only GUIDED autonomous control was broken.**
- *Fix:* **abandon GUIDED** → drive by direct **RC override in MANUAL** via `cmd_vel_to_rc` (mimics the TX). Diagnosed by logging `/mavros/rc/out` servo PWM per teleop primitive.

**E6 — Backward moves forward** ✅
- *Symptom:* backward teleop command → rover drives forward.
- *Cause:* GUIDED treats a negative-x velocity as a *direction to face* (turn around), not reverse.
- *Fix:* direct RC override sends a true reverse throttle PWM.

**E7 — "Smooth 360°" was actually a runaway** ⓘ
- *Symptom:* an earlier continuous smooth 360° looked like a success.
- *Cause:* it was the positive-feedback steering runaway **saturating** into a continuous spin — it can never converge to a target heading, which is why controlled/backward turns still failed.

---

## F. Command Pipeline & RC Override

**F1 — RC override silently ignored by ArduPilot** ✅  ← the "nothing moves" blocker
- *Symptom:* `/mavros/rc/override` publishes correct PWM but the servos don't respond.
- *Cause:* `SYSID_MYGCS=255`, but MAVROS sends as system id **1** → ArduPilot discards overrides from a non-matching GCS.
- *Fix:* `SYSID_MYGCS 255→1`.

**F2 — Pivot wrong direction under RC override** ✅
- *Symptom:* `+angular.z` pivots right.
- *Cause:* `SERVO1_REVERSED=1` mirrors the output (correct for the TX); our override sign was opposite.
- *Fix:* `steer_reversed=true` in `cmd_vel_to_rc` (fix the sign in OUR node, leave `SERVO1_REVERSED` alone so the TX stays correct).

**F3 — `cmd_vel_to_rc` E-stop LATCHES** ✅
- *Symptom:* after an E-stop test the robot won't move even after releasing.
- *Cause:* the node latches `estop_active=true` until it receives an explicit `/estop false` (no timeout).
- *Fix:* `estop_clear.sh` (`/estop false` + restore drive mode).

**F4 — teleop_twist_keyboard publishes nothing** ✅
- *Symptom:* node is running but `/cmd_vel_teleop` is silent.
- *Cause:* it only publishes while a key is **held in the focused terminal** (blocking `getKey`); finicky over SSH.
- *Fix:* use `ros2 topic pub` for deterministic tests.

**F5 — Duplicate cmd_vel_to_rc / twist_mux instances** ✅
- *Symptom:* 2+ nodes, conflicting output, odd publish rate.
- *Cause:* user launched `rc_control` while Claude also ran a standalone node.
- *Fix:* kill all, run exactly one; agree who launches the pipeline.

**F6 — ACRO rate loop oscillates** 🔴
- *Symptom:* a steady 0.4 rad/s command makes the rover swing right / 90° left / right.
- *Cause:* `ATC_STR_RAT` gains (P=0.2, I=0.2) unstable — large swings look like an integrator limit-cycle.
- *Next:* drop `I` hard (→~0.05), then `P` (→0.1), retest.

---

## G. Safety / E-stop

**G1 — Disarm via MAVROS refused** 🟡
- *Symptom:* `ros2 service call /mavros/cmd/arming "{value: false}"` doesn't disarm.
- *Cause:* ArduPilot refuses (thinks it's driving / needs a force flag).
- *Fix:* disarm via Mission Planner, or use mode HOLD; laptop E-stop = `estop.sh` (HOLD + streamed `/estop true`).

**G2 — RC transmitter (TX) is dead** 🟡
- *Symptom:* no hardware kill available.
- *Fix:* laptop E-stop scripts — `estop.sh` (HOLD + `/estop`, triple-coverage: FCU HOLD + cmd_vel_to_rc neutral + twist_mux pri-255 lock); Mission Planner Disarm/HOLD as backup.

---

## H. Diagnostic pitfalls (our own debugging mistakes)

**H1 — Analysis `awk` read the wrong odom columns** — produced a false "heading stuck at 0"; fixed by using the correct field indices and re-extracting. *Lesson: sanity-check the parser before trusting a scary conclusion.*

**H2 — `pkill` self-kill** — `pkill -f "<name>"` matched the pkill command's own cmdline and killed its shell / timed out. *Fix:* bracket trick `pkill -f "[n]ame"`, and don't put the target name in the same compound command that launches it.

**H3 — Background monitor killed by its own embedded pkill line** — launching `pkill …; python x.py` in one background command self-terminated. *Fix:* launch cleanly, kill separately.

**H4 — `/tmp` scripts lost on reboot** — `estop.sh` etc. vanished after a Jetson reboot (FCU params persist, `/tmp` does not). *Fix:* recreate from memory; consider a persistent scripts dir.

**H5 — First git commit included the colleague's dirs + junk** — `git rm -r --cached` silently failed (a redirect hid the error). *Fix:* `git rm -rf --cached`, and verify the staged list before committing.

**H6 — Cross-user DDS: topics visible but ZERO messages** — monitors run as `nvidia`, MAVROS runs as `jetson`. `ros2 topic list`/`info` showed "Publisher count: 1", but every `echo` delivered nothing and hung. *Cause:* FastDDS default **shared-memory** transport uses `/dev/shm/fastrtps_*` segments owned by `jetson`; a cross-user subscriber discovers over UDP multicast but cannot attach to SHM. *Fix:* force UDP-only via `FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/dagger_reports/tools/udp_only.xml`. *Lesson: "publisher exists but no data" across users = SHM permissions, not QoS.*

**H7 — Correlating separate log files by LINE NUMBER produced a false conclusion** — comparing `t_sp_lin.log` (30Hz) against `t_rcout.log` (2Hz) by offset made dead time between keytaps look like "the FCU outputs nothing on a valid command" — **wrong**. *Fix:* `tools/chain_monitor.py` samples every link at 10Hz into ONE timestamped CSV with **age** columns. *Lesson: never correlate async streams by line index; timestamp them in one process.*

**H8 — Recommended teleop params that don't exist in this build** — advised `-p repeat_rate:=10.0 -p key_timeout:=0.5`; Humble's `teleop_twist_keyboard.py` only declares `stamped, frame_id, speed, turn`. Undeclared `-p` overrides are silently ignored → wasted run. *Fix/lesson:* grep `declare_parameter` in the installed module before recommending params.

**H9 — Scratchpad under `/tmp` wiped mid-session** — monitor + CSV + XML vanished (disk was 85% free → a tmp-cleaner, not disk pressure). *Fix:* test tooling now lives in `/home/jetson/dagger_reports/tools/` (durable, outside the shared workspace). Generalises H4.

**H10 — Diagnosing a control loop through a 2Hz keyhole** ⚠️ *the big one* — `/mavros/rc/out` streams at **2Hz** (`SR0_RC_CHAN=2`), which by Nyquist resolves nothing above 1Hz. A fast control signal aliased into a fake "±226µs square wave," and **three separate root-cause theories were built on that artifact** (heading control / cmd_vel_to_rc interference / EKF failsafe) — all disproven. At **25Hz** the same signal was a **smooth ramp settling at ch1=1301**. *Fix:* `ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 511, param1: 36.0, param2: 40000.0}"` (SERVO_OUTPUT_RAW @25Hz) and sample ≥2× that. Note `SR0_*` params are **inert** — MAVROS overrides stream rates on connect. *Lesson: check the sample rate of your instrument BEFORE forming a theory about the waveform.*

**H11 — `pkill`/`pgrep -f` self-kill (recurring)** — a pattern like `pgrep -f "chain_[m]onitor.py"` still matches the parent shell when the **literal** target string appears elsewhere in the same compound command (e.g. the `nohup python3 .../chain_monitor.py` launch line). Killed the shell repeatedly (exit 143/144, no output). *Fix:* bracket trick **and** put the kill in its own separate call — never in the same command as the launch.

---

## I. GUIDED teleop investigation (2026-07-12) — instrumented command→PWM trace

Method: `tools/chain_monitor.py` logs teleop → twist_mux → cmd_vel_safety → setpoint → `/mavros/rc/out` PWM → IMU at 10Hz on one timeline, with staleness ages. Run it with the UDP-only profile (H6).

**I1 — Setpoint FLICKERS 0.5 → 0 → 0.5 at ~1Hz; rover lurches** ✅ *cause identified*
- *Symptom:* erratic stop-start motion under keyboard teleop; `ch3_thr` bouncing **1509 ↔ 1668**.
- *Cause:* `teleop_twist_keyboard` publishes **only on keypress** (see F4). Tapping ~1×/s leaves ~1s gaps; `cmd_vel_safety`'s `command_timeout = 0.5s` (cmd_vel_safety.py:19,68) correctly zeroes the setpoint in every gap.
- *Proof:* `sp_lin` drops to 0.0 exactly when `teleop_age` crosses 0.5s; snaps back to 0.5 on the next tap.
- *Fix:* **do NOT weaken the 0.5s watchdog** — it is correct for a 350kg vehicle. Stream instead: `ros2 topic pub -r 10 …`, or **hold** the key (terminal auto-repeat) rather than tapping. This build has no `repeat_rate` (H8).
- *Also affects the RC path:* `cmd_vel_to_rc` has the same 0.5s `command_timeout`.

**I2 — GUIDED straight-line drive is HEALTHY** ✅ *corrects an earlier wrong conclusion*
- With a gap-free 10Hz `linear.x=0.3`: `sp_lin` rock-steady; `ch3` ramps 1500→**1619**, settles ~1558 (normal overshoot+settle); `ch1` stays **1500–1502** (no phantom steering); on release `ch3` drops **below** neutral (1473→1446) = correct active braking. User: *"forward was decent."*
- *The earlier claim "the FCU produces no throttle in GUIDED" was WRONG* — it measured dead time between keytaps (H7).

**I3 — Pivot instability: steering inverted in every CLOSED-LOOP mode** ✅ *root cause found*
- *Symptom:* user: *"run was right left and so on."* Commanded steady `angular.z=+0.5` (→ `sp_ang=-0.5` after `invert_yaw`); `ch1` **saturated, slamming 1000 ↔ 2000**, `imu_gz` peaked **+1.8 rad/s vs 0.5 commanded** (3.6×, WRONG direction).
- *Cause:* **`SERVO1_REVERSED = 1`**. It was set so the **TX** felt correct (see F2), but it reverses the steering **output** while the gyro is untouched → in any mode with feedback the loop becomes **positive feedback**: controller steers right → vehicle turns left → error grows → steer harder → full lock → runaway.
- *Why MANUAL always worked:* MANUAL has **no feedback loop** — the reversal merely makes the stick feel right. Every closed-loop mode (ACRO, GUIDED) inherited an inverted loop. **This is the single root cause behind E7 ("smooth 360° was a runaway"), F6 (ACRO oscillation / "wrong direction"), and I3.**
- *Fix:* **`SERVO1_REVERSED = 0`** (done 2026-07-12) → saturation gone immediately; the wound-up steering offset (ch1 stuck at 1554) also released.
- ⚠️ **Consequence:** TX steering is now **inverted in MANUAL**. Proper fix is `RC1_REVERSED = 1` (reverse the INPUT, which no control loop depends on). **NOT YET APPLIED.**
- *`invert_yaw=True` in `cmd_vel_safety` is a band-aid on this same bug* — it flips the commanded direction so the rover *looks* right while feedback stays positive. Remove it once the sign is fixed end-to-end.

**I3b — GUIDED turn-rate control WORKS once the sign is fixed** ✅
- With `SERVO1_REVERSED=0, FF=0.8, P=0.1, IMAX=0.1` and a gap-free 10Hz command, the FCU tracked **perfectly for ~5s**: `ch1` steady at **1301**, `imu_gz` locked on **−0.5** (commanded −0.5). At 0.25 rad/s: `imu_gz` −0.22. **The tune is good.**
- *Proven NOT heading control:* `nav_bearing`/`target_bearing` stayed **0** for every pivot → Rover is in the **turn-rate submode**. (An earlier entry claimed the opposite — that was wrong.)

**I4 — Mode changes: TX switch owns the mode; HOLD does not come from the TX** ✅/🔴
- `MODE_CH=5`; **MODE1–MODE5 = 0 (MANUAL), MODE6 = 15 (GUIDED)**. ch5 PWM → slot: ≤1230→M1, 1231-1360→M2, 1361-1490→M3, 1491-1620→M4, 1621-1749→M5, ≥1750→M6.
- *Observed:* ch5 **1495 → MANUAL**; ch5 **1945 → GUIDED**.
- *Cause of the "drift":* setting GUIDED over **MAVROS while the switch still said MANUAL** — the two fight and the switch wins. **Fix: select GUIDED with the TX switch (ch5→1945).** Verified: 30s, 1504 samples, **zero transitions**.
- 🔴 **STILL OPEN — `HOLD`:** HOLD (4) is in **no** MODE slot, so the switch *cannot* select it. Those HOLD events are a genuine **failsafe** (`FS_ACTION=2`, `FS_EKF_ACTION=1`). User independently confirmed HOLD is not from the TX. **Investigate tomorrow.**
- *Suggestion (not applied):* set `MODE5=4` to give the switch a real hardware HOLD/abort position.

**I5 — Sensors are HEALTHY (ruled out)** ✅
- GPS: **17 satellites, fix_type 4 (DGPS), eph 0.85m** — good. (`GPS_TYPE` reads "not set" because 4.5+ renamed it **`GPS1_TYPE`** = 9/DroneCAN.)
- Gyro: `imu_gz` sign and magnitude **matched the observed physical rotation every run**. Good.
- EKF failsafe: never announced (0 statustext, no mode change during runs) — though sampling was only in short windows.

**I6 — MOST OF TODAY'S PIVOT DATA IS CONTAMINATED** ⚠️
- Mode was flipping GUIDED/MANUAL/HOLD *underneath* the tests while conclusions were drawn about control tuning. A 35s "FCU does nothing" window was simply the vehicle sitting in HOLD.
- The "~5s periodic sign flip" (amplitude = `FF × desired`, **proven time-based not angle-based** by halving the command: ±225µs @0.5 → ±113µs @0.25) is **most likely the vehicle dropping out of GUIDED**, not a control-loop or Rover reverse-ambiguity issue. **Re-test from a stable GUIDED baseline before theorising further.**

---

## FCU params CHANGED on 2026-07-12 (all verified by force-pull readback)
| Param | Was | Now | Why |
|---|---|---|---|
| `SERVO1_REVERSED` | 1 | **0** | fixes inverted closed loop (I3). ⚠️ **TX steering now reversed in MANUAL** |
| `ATC_STR_RAT_IMAX` | 1.0 | **0.1** | integrator could command full lock |
| `ATC_STR_RAT_P` | 0.3 | **0.1** | let FF carry the rate |
| `SR0_RC_CHAN` | 2 | 25 | **inert** — MAVROS overrides SRx on connect |
| SERVO_OUTPUT_RAW rate | 2Hz | **25Hz** | via `MAV_CMD_SET_MESSAGE_INTERVAL` (msg 36 @ 40000us). **NOT persistent — re-send after reboot** |

Unchanged: `ATC_STR_RAT_FF=0.8` (measured correct), `ATC_STR_RAT_I=0.05`, `FS_THR_ENABLE=0`, `FS_GCS_ENABLE=0`, `RC1_REVERSED=0`.

---

## Summary counts (2026-07-12, updated)
- ✅ Solved: ~27 · 🟡 Workaround/partial: ~4 · 🔴 Open: 2 (I4 HOLD failsafe source, C3 duplicate-odom?)
- **Biggest win:** **I3 — `SERVO1_REVERSED=1` was inverting every closed-loop mode.** One parameter behind E7, F6 and I3. GUIDED/ACRO were never "structurally incapable"; the loop was wired backwards.
- **Second win:** I1 — teleop watchdog flicker (teleop publishes only on keypress vs a 0.5s watchdog).
- **Third:** I4 — the TX switch owns the mode; commanding GUIDED over MAVROS fights it.
- **Lesson (H7/H10):** three wrong theories came from a **2Hz** `rc/out` stream; at 25Hz the "square wave" turned out to be a smooth ramp. **Fix the instrument before theorising.**
- **Tomorrow:** (1) `RC1_REVERSED=1` before anyone drives with the TX; (2) find what fires HOLD; (3) clean pivot test from a TX-selected GUIDED baseline; (4) then Nav2 turn.
