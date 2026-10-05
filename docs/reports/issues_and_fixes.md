# Dagger UGV — Issues & Fixes Log (running report)

Private running log of every problem hit on the Dagger project — symptom, root cause,
fix, and status. **Not** in the shared workspace. To be finalized at project end.

Owner: Abhinav · Location: `/home/jetson/dagger_reports/` (group `dagger`, writable by jetson + nvidia)
Last updated: 2026-09-26 (Q27 LiDAR hold; section Q added — follow-me: follow-path collision monitor, Humble approach-polygon trap, detector/camera gaps)

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

---
---

# PART 2 — 2026-07-24 → 2026-09-25

> Gap note: nothing was logged here between 2026-07-13 and 2026-09-17. From 2026-09-14 Claude ran as
> `nvidia` and this folder was owned by `jetson` (not writable), and that was not flagged. Entries below
> were back-filled on 2026-09-17 from session notes, git commits and test bags. Bags: `~nvidia/dagger_bags/pf_*`.

## Status updates to earlier entries
| Entry | Was | Now | Resolution |
|---|---|---|---|
| C3 heading bistable −176°/0° | 🔴 | ✅ | Colleague's duplicate EKF on the shared ROS_DOMAIN_ID 0 (his machine). Gone since the colleague left the project (07-25). |
| E4 stick-slip / bimodal rotation | 🔴 | ✅ | `ATC_STR_RAT_FF` 0 → 0.8 carries the tracks through static friction (07-25). |
| E5 "GUIDED cannot turn this vehicle" | ✅ | ⓘ superseded | Real causes were `SERVO1_REVERSED=1` (I3), car-style output mixing (J1) and GSF yaw (J4). GUIDED now turns fine. |
| F1/F2 RC-override path | ✅ | ⓘ retired | User rule 07-25: **no RC override**. Control is GUIDED `setpoint_velocity` only; `rc_control`/`cmd_vel_to_rc` and ACRO dropped. |
| F6 ACRO rate loop oscillates | 🔴 | ⓘ moot | ACRO needs RC override → path abandoned. |
| I4 mystery HOLD | 🟡 | ✅ | `FS_THR_ENABLE=1` + `FS_ACTION=2` with the flaky/dead TX → RC failsafe → HOLD. Still set today (see P4). |
| "RC1_REVERSED=1 before TX use" | todo | ⓘ moot | Superseded by skid-steer conversion (J1); TX is dead anyway. |
| D4 turn overshoot/hunt | 🟡 | ✅ | Solved in stages: J1, J2, K1, M2, M3, N4. 180° goal heading now lands at 0.0° error. |

---

## J. Drivetrain & ArduPilot yaw (2026-07-24/25)

**J1 — GUIDED in-place pivot bang-bang (car-style mixing)** ✅
- *Symptom:* GUIDED pivots slammed steering full-scale both ways.
- *Cause:* FCU in steering+throttle output (`SERVO1=26`, `SERVO3=70`) into a motor controller that mixes internally; the ground-steering output path reverses steering with speed sign.
- *Fix:* ArduPilot **skid steer** `SERVO1_FUNCTION=73` (ThrottleLeft), `SERVO3_FUNCTION=74` (ThrottleRight) + an **STM32 demixer board** that undoes the motor controller's internal mixing.

**J2 — `mav_frame` must be BODY_NED** ✅ (see P1 for the recurrence)
- *Symptom:* commanded twist executed in the wrong direction once heading ≠ 0.
- *Fix:* `ros2 param set /mavros/setpoint_velocity mav_frame BODY_NED`. **Resets on every mavros restart** → now done (and verified) by `tools/preflight.py`.

**J3 — `cmd_vel_safety invert_yaw` double-inverted Nav2** ✅
- Band-aid from the SERVO1_REVERSED era; after I3 it made Nav2 turn the wrong way (positive feedback). Default True → **False**.

**J4 — GUIDED self-rotates at rest with zero command (compassless GSF yaw)** ✅ (by K0)
- *Symptom:* ~128° uncommanded rotation at rest; turn hunting worst during in-place pivots.
- *Cause:* `COMPASS_ENABLE=0` + `EK3_SRC1_YAW=8` (GSF) → yaw observable only while translating. EK3 reported a 0.11 rad/s phantom rate at rest (`/mavros/imu/data`), while the raw gyro (`/mavros/imu/data_raw`) read 0.0. ArduPilot's rate loop steered to null the phantom.
- *ROS-side fix:* our EKF fuses `/mavros/imu/data_raw` (rest drift 6.2 → 0.0 °/s).
- *FCU-side:* ExtNav-yaw relay built as a workaround, then made unnecessary by K0.

**J5 — Legacy packages removed** ⓘ — `dagger_controller`, `dagger_slam` deleted and purged from git history (07-25). First-party now: `dagger_bringup`, `dagger_description`, `msgs_comm`. (`install/` may still hold stale copies.)

## K. Dual-antenna GNSS (2026-09-11)

**K0 — Yaw saga resolved in hardware: CUAV C-RTK 2HP dual-antenna GNSS** ✅
- Moving-baseline heading works **at rest and during pivots**; `EK3_SRC1_YAW=2`. Noise 0.2° vs 6.2 °/s drift before.
- Cube moved `ttyACM0` → `ttyACM1` on USB re-enumeration → `fcu_url` now uses the stable `/dev/serial/by-id/usb-CubePilot_CubeOrange+_…-if00` path.
- **First clean 180° in-place turn** via Nav2 `/spin` (0 command reversals).

**K1 — "Extrapolation into the future" TF errors → erratic motion** ✅ — EKF `predict_to_current_time: true`, frequency 30 → 20 Hz.

**K2 — KISS-ICP position jumps during rotation** ✅ — 16-beam scan matching degenerates while spinning. KISS removed from the Nav2 odom path (see L1).

**K3 — Cross-user DDS returned (Claude as nvidia, ROS as jetson)** ✅ — permanent fix: run the ROS stack as `nvidia` too (native SHM). `tools/udp_only.xml` kept only as a fallback.

## L. Path following: odometry & planning (2026-09-14)

**L1 — Rover arced left on a straight goal (GPS-anchored odom)** ✅
- *Symptom (bag `pf_legA_base_straight_2`):* EK3 pose (our odom via `fcu_odom_relay`) slid **1.7 m right** in 20 s, and **3.2 m at rest** in 50 s; Nav2 steered left to "correct" the phantom offset.
- *Cause:* non-RTK GPS position corrections move the EK3 pose with zero velocity. Mission Planner AUTO tolerates it because ArduPilot tracks inside the same drifting frame.
- *Fix (REP-105 split):* `odom_source:=fused` — robot_localization EKF on **EK3 body velocity + raw gyro rate + dual-antenna heading**, no GPS position in odom. GPS position only in `map→odom` (`gps_map_odom`). Commit `eda7b2b`.

**L2 — Planner bulged 45° diagonals on straight goals** ✅ — SmacPlanner2D 8-connected grid → **Theta\*** (any-angle). `eda7b2b`.

**L3 — Lethal "ground ring" 4–5 m ahead → 14 m detour** ✅
- *Cause:* URDF put the LiDAR at 1.30 m; measured **1.12 m** → flat ground read +0.19 m, above p2l min_height 0.15.
- *Fix:* `lidar_joint` z 1.1 → 0.92; scan band **0.35–0.75 m** (the upper limit keeps mast self-returns at 0.81–1.12 m out). `eda7b2b`.
- ⚠️ Stopgap: obstacles **under 35 cm are invisible** (curbs, debris) → ground segmentation still open (P2).

**L4 — Fused odom crept 0.3 m/min at rest** ✅ — EK3 5 mm/s velocity bias → `zero_velocity_deadband 0.02` in the relay.

**L5 — Waypoint abort + recovery spin/backup near a wall** ✅
- *Cause:* RemovePassedGoals looks up `map` (absent in odom-only mode) → TF error → default BT recovery spun 96°, backed up 0.3 m, spun again with a wall 0.5 m away.
- *Fix:* custom BTs `behavior_trees/dagger_navigate_{to_pose,through_poses}.xml` **without Spin/BackUp** (clear costmaps + Wait); identity map→odom placeholder for odom-only tests. `17816b5`.

**L6 — 14 s stall mid-route (RotationShim stiction deadlock)** ✅ — shim `closed_loop: false` (rate-limit vs last command, not measured velocity) + `angular_disengage_threshold 0.3`. `17816b5`.

**L7 — FCU barely executes small turn rates while driving** ✅
- actual/cmd gain 0.11 @0.02 rad/s … 0.70 @0.34 rad/s (track lateral friction).
- *Fix (user-set FCU):* `ATC_STR_RAT_I` 0.05 → 0.2, `P` 0.2 → 0.3. Lane change route error 0.41 → **0.14 m RMS**. Real footprint ±0.85 × ±0.58 m. `51897e2`.

**L8 — Uncommanded +32° lurch after small turn commands** ✅ — I=0.2 integrator wound up against stiction. User set `ATC_STR_RAT_IMAX` 0.4 → **0.2**.

**L9 — pointcloud_to_laserscan wedges → `/scan` 0 Hz → costmaps blind** 🟡
- Humble lazy-subscribe bug: subscriber count flaps at costmap startup and the node stops releasing scans.
- *Workaround:* `scan_watchdog` node (permanent subscriber; kills p2l if /scan silent > 4 s while clouds arrive) + `respawn=True`. Preflight gates /scan rate. `75c19b6`.

**L10 — Goals aborted after ~11 s** ✅ — goal stamps aged out of the 10 s TF buffer → stamp 0 (latest TF) in `send_goal.py` / `gps_goal.py`.

**L11 — "Confused" GPS-mode run: 3-minute in-place hunt** ✅
- *Cause:* costmaps in `map` while map→odom slid 3 m/90 s at rest (8.5 m during pivots) → remembered obstacles slid → ghost walls → path flipped every replan.
- *Fix:* `nav2_gps.launch.py` keeps costmaps/planning in **odom**; GPS only converts lat/lon goals. `75c19b6`.

**L12 — Odom yaw drifted ~7° over 3 min of pivoting** ✅ — new `gps_heading_imu` (GPS_RAW yaw → absolute yaw into the EKF, gated while |gyro z| > 0.1 rad/s because GNSS heading lags in fast spins). `75c19b6`.

## M. Middleware, oscillation, final approach (2026-09-16)

**M1 — LiDAR degraded 10 → 2.7 Hz, clouds up to 1.2 s late** ✅
- *Cause:* 0.8 MB clouds > Fast-DDS default SHM segment → fragmented onto UDP loopback → dropped by 212 KB kernel socket buffers; rslidar queue 100 RELIABLE.
- *Fix:* `config/fastdds_dagger.xml` (64 MB SHM segment, `maxMessageSize` 4 MB), exported in nvidia's `.bashrc`; rslidar `ros_queue_length` 5; host `net.core.rmem_*` 32 MB (`tools/setup_host.sh`, persisted in `/etc/sysctl.d/60-dagger-net.conf`).
- *Result:* **10.1 Hz, 111 ms latency, 0 socket drops.** `1d1fbb7`.
- ⚠️ Gotcha: the XML tag is camelCase `maxMessageSize`; `max_message_size` silently invalidates the whole transport block.

**M2 — Pivot → fake odom translation → costmap smear → backwards replan → runaway** ✅
- *Symptom (bag `pf_lane_radius_fix`):* 633° of spin, odom wandered ~1 m while parked.
- *Cause:* during an in-place pivot EK3 body velocity is dominated by the GNSS antenna lever arm → up to 0.9 m fake translation → obstacles shift → looks boxed in → Theta* routes backwards → shim pivots again.
- *Fix:* `pivot_yaw_rate_gate 0.25` (zero linear velocity while |ωz| > 0.25; gyro carries heading); shim threshold 1.2 rad (arc instead of pivot mid-route); RemovePassedGoals radius 0.7 → 2.0 (a passed waypoint made the path double back). `62ce1f8`.

**M3 — ★ Oscillation root cause: shim sampled the path at 0.5 m** ✅
- Theta* can leave the pose backwards for ~1 m when it detours; at 0.5 m the shim read −147° and pivoted ~180°. The same path read +28° at 2 m.
- *Fix:* `forward_sampling_distance` 0.5 → **2.0**. Reversals 42–104 → 1; 50° turn now driven as an arc. `36865a1`.

**M4 — End-of-goal yaw swing (57° in the last metre)** ✅ — pure pursuit ω ∝ 1/dist² while chasing the last 0.25 m. `xy_goal_tolerance` 0.25 → 0.4, `min_approach_linear_velocity` 0.15 → 0.10, `approach_velocity_scaling_dist` 1.0 → 1.5. `9f81ccc`.

## N. Foxglove, safety check, straight-line & turn accuracy (2026-09-17)

**N1 — ⚠️ SAFETY: preflight left `mav_frame=LOCAL_NED`** ✅
- The BODY_NED set call silently timed out after a mavros restart → the FCU would read Nav2's body twist in the earth frame (wrong-direction drive).
- *Fix:* `preflight.py` retries and **verifies by read-back**; FAIL blocks the run. `01170c3`.
- *Rule:* after ANY mavros/T1 restart → full preflight with config before any goal.

**N2 — Foxglove goal sending, locked down** ✅ — `launch/foxglove.launch.py`: clients may publish only `/goal_pose` and `/estop`; no `/mavros` services/params; point cloud off unless `with_cloud:=true`. `945c4ee`.

**N3 — Costmap obstacles too fat; open directions never cleared** ✅ — inflation 1.0 → **0.8**, `cost_scaling_factor` 6.0; scan `inf_is_valid: true` so max-range (∞) rays clear space (Foxglove showed 207 ∞ values). `e1c05b1`.

**N4 — Straight run curved + 180° goal turn ended 17° short** ✅
- *Cause 1:* EK3 body **vy up to −0.17 m/s** against +0.22 m/s forward (a fake 37° crab) while accelerating with gyro 0 → fused odom slid 1 m right → Nav2 steered a real left curve.
- *Fix:* `zero_lateral_velocity: true` (tracked rover is non-holonomic; vy published as 0).
- *Cause 2:* `yaw_goal_tolerance` 0.4 rad (23°) accepted the short turn → **0.2**. `c5f43d7`.

**N5 — Lookahead 2.0 → 1.5 m** ✅ — 6 m straight + 180° (bags `pf_straight6_turn180_nonholo` vs `_la15`):
| | lookahead 2.0 | **lookahead 1.5** |
|---|---|---|
| max offset / RMS | 0.181 / 0.104 m | **0.168 / 0.102 m** |
| heading max | 10.5° | **7.1°** |
| 180° turn error | −1.5° | **0.0°**, 0 reversals, ~6.4 s |
`3973a6e`.

**N6 — Residual straight-line drift = left/right TRACK IMBALANCE** 🔴 *deferred by user (2026-09-17)*
- *Symptom:* ~0.10 m RMS drift to the rover's **own right in both travel directions** (so not slope).
- *Evidence:* straight cruise, gyro ≈ 0: left ch1 1640 µs vs right ch3 1684 µs (FCU steering integrator carries ~37 µs). Across 26 bags, left/right deflection ratio: 0.92 @0.11 m/s, 0.87 @0.16, ~0.72 @0.31–0.34, 0.77 @0.41 m/s; through-origin fit **0.82** → proportional (range), not trim.
- FCU servos are symmetric: SERVO1/3 = 1100/1500/1900, not reversed.
- *Planned fix (in ArduPilot, user's choice):* `SERVO1_MAX` 1900 → **1830** (left forward range ×0.825), then repeat the 6 m + 180° test. Still right → 1800; now left → ~1860. Undo = 1900.
- Caveat: some 09-14/15 bags showed the opposite sign (lane slope?) → start with a partial correction. Pivots become slightly asymmetric (rate loop corrects).

**N7 — No independent safety layer between Nav2 and the FCU** ✅ *built, static + live validated (2026-09-17)*
- *Gap:* only the costmaps/planner avoided obstacles; a planner/odom fault had nothing behind it.
- *Built (`be024b7`):* `nav2_collision_monitor` on `/cmd_vel_nav_raw → /cmd_vel_nav` — stop zone 0.6 m ahead of bumper (±0.70), slow ×0.4 to 1.75 m (±0.85), footprint approach 1.5 s. Dedicated `/scan_safety` (0.30–1.60 m, range_min 1.0 m to drop mast returns) + its own watchdog. Teleop not gated (RC is the manual override).
- *Humble pitfall found:* a stale observation source silently counts as "no obstacles" → `cmd_vel_safety fresh_topic` zeroes all output if `/scan_safety` > 0.5 s old.
- *Static walk test (isolated copy, live LiDAR, 2026-09-17):* 14 min, 0 false triggers; person 1.2–1.35 m ahead → 0.40→0.16; person at bumper → 0.00 held for 1 min; resume within 1 s. Person ~0.9 m left of centre ahead of the corner → only brief slow blips (outside ±0.70 stop width — correct).
- *Live solo test after N8 fix (bag `pf_cm_solo_step_in`, 15:20, user stepped in twice, goal sent by a user-run gated timer script):* encounter 1 — slow at 1.87 m gap, **stop at 0.56 m, rolled 0.01 m, held 4 s (min gap 0.46 m)**, resumed after user stepped aside; encounter 2 — slow at 1.31 m, **stop at 0.51 m, rolled 0.02 m (min gap 0.48 m)**, resumed. Goal succeeded (7.67 m). Transient "start or goal pose are an obstacle" replan abort while the person was close — BT recovered.
- ⚠️ *Limits:* LiDAR near-field blind zone — at the bumper only objects > ~1 m tall are seen (0.5 m tall at 1.75 m out); low obstacles vanish when close and the monitor does not latch. Points within 1.0 m of hull centre (~0.15 m of bumper) are dropped by range_min.

**N8 — ⚠️ Rover coasted 0.78 m after a zero-velocity command (live collision-monitor test)** ✅ *fixed by cruise calibration (user-set 2026-09-17)*
- *Test (bag `pf_cm_live_step_in`, 2026-09-17 14:59):* 8 m straight, helper stepped in. Collision monitor correct: slow at 1.53 m gap (14:59:25.7), **stop at 0.52 m gap (14:59:28.2)**, FCU setpoint 0.00 from then on.
- *Failure:* rover kept driving **0.78 m over 4.5 s** (v 0.29 → 0.13 m/s, track PWM ~1620/1680 → 1530/1580). User **disarmed with RC at 14:59:32.7**; helper ended ~0.25 m beside the front-right corner (rover had also steered left around them). Near miss.
- *Cause:* ArduPilot speed controller carries the cruise throttle in the **integrator**: `CRUISE_SPEED 2.0 / CRUISE_THROTTLE 50` → feed-forward at 0.4 m/s = 10 % throttle, but ~45 % is needed; the other ~35 % sits in `ATC_SPEED_I` (0.2), which unwinds at only ~6 %/s when the target drops to 0 (`ATC_SPEED_P` 0.2 adds only −6 %).
- *Proposed fix:* calibrate cruise so feed-forward carries the load — `CRUISE_SPEED 0.4`, `CRUISE_THROTTLE 45` (from 26 bags: ~45 % output at 0.41 m/s), or RC aux "Learn Cruise" (option 50) while driving steady at 0.4 m/s. `WP_SPEED` is explicitly 2.0 and `RTL_SPEED` 0 → AUTO speeds unaffected. Then repeat: zero-command stop distance, then helper test.
- *Result (bag `pf_stopdist_cruise04`, user set CRUISE_SPEED 0.4 / CRUISE_THROTTLE 45):* goal cancelled at 0.37–0.42 m/s → **rolled 0.14 m, at rest in 0.70 s** (was 0.78 m / 4.5 s at 0.29 m/s). Track PWM back to neutral within 0.5 s.
- *Side finding (🟡 open):* after the hard stop EK3 body velocity read −0.07 m/s decaying over ~8 s with PWM at neutral → fused odom slid ~0.3 m backwards (estimator lag, not motion; 0.02 deadband too small). Candidate fix: zero-velocity update in `fcu_odom_relay` when track PWM is neutral.
- *Also explains:* the old "resting creep" and slow final stops (Nav2's slow approach hid it).

## Q. Object following (follow-me) — 2026-09-25

**Q0 — Scope** ⓘ — user chose follow-me over SLAM. Agreed order: (1) collision monitor on the follow command path, (2) D435 bring-up, (3) TensorRT detector without PyTorch, (4) target selection with LiDAR range, (5) follow controller, (6) disarmed tracking test then a slow walking test on the RC. Ground segmentation (P2) moves to next-after-follow-me.

**Q1 — Follow commands would have reached the motors with NO safety gate** ✅ *built 2026-09-25, bench test pending*
- *Gap:* the collision monitor (N7) only filters the Nav2 path (`/cmd_vel_nav_raw → /cmd_vel_nav`). `twist_mux` already carried a `follow` slot (`cmd_vel_follow`, priority 30) wired straight through to `cmd_vel_safety` → mavros. A follow controller publishing it would have driven 350 kg at a person with no stop and no slow zone.
- *Built:* `params/collision_monitor_follow.yaml` — a second `nav2_collision_monitor` instance, `/cmd_vel_follow_raw → /cmd_vel_follow`, deliberately the **same zones and the same `/scan_safety`** as the Nav2 monitor so the rover behaves identically near a person whichever path drives. New `launch/follow.launch.py` = **T4**, with its own lifecycle manager: follow-me starts and stops without touching Nav2, and a follow node that fails to activate cannot stop Nav2 coming up.
- *Verified offline:* configures + activates, all three polygons and the scan source created, no orphan nodes.
- *Tools:* `tools/follow_check.py` (read-only wiring check) · `tools/follow_gate_test.py` (stands in for the controller: publishes a steady `/cmd_vel_follow_raw` so the zones can be walked into — **REFUSES to start unless the FCU is disarmed** and stops publishing if it arms mid-run).
- *Design decision — no reverse in follow mode:* a stop trigger zeroes the whole command including reverse, and backing a 350 kg hull with no rear sensing is the worse option. If the target closes inside the stop zone the rover holds still until they move away.
- ⚠️ *Inherits N7's blind zone:* at the bumper the LiDAR sees only objects taller than ~0.97 m. A standing adult yes, a crouching one no. Test subjects stay standing.

**Q2 — ⚠️ Humble: an "approach" polygon fails OPEN, and silently** ⓘ *(found while writing Q1)*
- `nav2_collision_monitor` 1.1.20 `polygon.cpp:314-324`: for `action_type: approach`, `getParameters()` reads `footprint_topic` and **returns before it ever reads `points`** — a static approach polygon is impossible. If that topic never publishes, `poly_` stays empty, `getPointsInside()` returns 0 forever, and **nothing is logged**.
- *Consequence:* `FootprintApproach` on **both** monitors is live only while Nav2's local costmap publishes `/local_costmap/published_footprint` → **follow mode requires T2 (`nav2_gps`) to be up**. The stop and slow polygons are static and unaffected.
- *Second trap:* `Polygon::publish()` emits an **empty** `PolygonStamped` rather than nothing, so "the topic is publishing" does not prove the gate works — the **vertex count** is the test. Both new tools check it.
- *Third trap:* `publishPolygons()` is only called from `process()`, which runs per incoming `cmd_vel` → the zone polygons do not publish **at all** until a command is flowing. That is why the polygon check lives in `follow_gate_test.py` and not in `follow_check.py`.

**Q3 — Both monitors' polygon logs say `collision_monitor`** ⓘ — the `Polygon`/`Source` classes use a hardcoded logger name, so `[PolygonStop]: Creating Polygon` looks identical for the Nav2 and the follow instance. Action messages ("Robot to stop") use the node logger and **do** distinguish `collision_monitor` from `collision_monitor_follow`.

**Q4 — Detector toolchain gaps** ✅ *closed 2026-09-25, `d472f20`*
- TensorRT 10.3 + `/usr/src/tensorrt/bin/trtexec` present. `torch`, `torchvision`, `ultralytics`, `onnxruntime` absent (deliberate — torch on JetPack is painful).
- **No CUDA memory allocator:** `pycuda` and `cuda` are both missing, so the TensorRT Python API cannot allocate device buffers — the plan does not work as written without one. *Verified fix:* `cuda-python==12.6.2.post1`, cp310 aarch64 wheel, 24.5 MB, matches the installed CUDA 12.6; binary wheel, no compilation. Install `--user` (stack and Claude both run as `nvidia`).
- **No way to export a model** (no torch) → fetch a pre-exported ONNX. Official Megvii YOLOX release assets verified reachable: `yolox_nano.onnx`, `yolox_s.onnx` (COCO, class 0 = `person`). Nano first; S is a drop-in on the same pipeline if range is poor. YOLOX ONNX emits raw grid predictions → decode + NMS in numpy.
- `vision_msgs` not installed (apt candidate `4.1.1-1jammy`) — **needs user sudo**.
- *Env for the record:* JetPack R36.5, CUDA 12.6, Python 3.10, cv2 4.5.4, numpy 1.21.5, 6 cores / 7.6 GB RAM / 186 GB free.
- *Built:* `dagger_bringup/yolox_trt.py` (engine load, cv2 letterbox, TRT 10 name-addressed IO, YOLOX grid decode, numpy NMS) · `tools/fetch_model.sh` (fetch + `trtexec` FP16 build; `models/` gitignored — an engine is tied to one GPU + TensorRT version) · `tools/detect_bench.py`.
- ⚠️ *Normalisation determined empirically, not guessed:* YOLOX changed its input normalisation between releases and the wrong choice yields **plausible garbage rather than an error**. On the reference `dog.jpg`, `legacy=False` (raw 0–255 BGR) finds dog 0.86 / bicycle 0.80 / car 0.80 at sane boxes; `legacy=True` (BGR→RGB, /255, ImageNet mean/std) finds **nothing at all**. `legacy=False` is now the default with the evidence recorded beside it; `detect_bench.py --legacy auto` re-derives it if the model ever changes.
- *Measured (yolox_nano FP16 416×416, Orin at the **15 W** power mode):* full `detect()` **15.76 ms median / 19.5 ms max = 63.4 FPS**; stages within it: preprocess 3.57, inference 9.95, decode 1.48. Flat out: GPU 57 %, CPU 18 % of 6 cores, 5.34 W. `trtexec` alone: 274 qps, 3.64 ms GPU compute.
- ⚠️ *The first benchmark under-reported by 2×* — it timed preprocess + inference + decode but **not** the class-score reduction and NMS that the node pays every frame (bench said 15 ms, the node measured 32). `detect_bench.py` now times the real `detect()` call too. Fixing the measurement exposed the cost: the per-class argmax ran over all 3549 anchors × 80 classes. Class scores are ≤ 1, so `score > conf` implies `objectness > conf` → gating on objectness first **discards nothing** and cut `detect()` from ~32 ms to 15.76 ms (detections byte-identical). `1ac360f`.
- *Headroom:* the follow loop needs 10 Hz (the LiDAR's rate) ≈ **16 % duty**. Live with the camera: camera + detector together **15.3 % of six cores, 8.9 % GPU**. Nav2 keeps its headroom. Raising `nvpmodel` above 15 W would improve this and cost everything else — user's call, not taken.
- ⓘ *In the node `detect()` measures 26.9 ms, not 15.76:* the bench runs flat out and holds a core at turbo, while a 16 ms job every 100 ms lets `schedutil` drop the core to 729 MHz. Still 27 % of one core at 10 Hz — worth knowing, not worth fixing.

**Q5 — Camera bring-up gaps** ✅ *closed 2026-09-25, `339c65a` + `d472f20`*
- `use_camera:=true` exists in `robot.launch.py` → `sensors.launch.py`, but **T1 is `drive_bringup.launch.py`, which includes neither** — it launches the LiDAR node directly. The flag is unreachable from the actual bring-up. → new `camera.launch.py` (colour only, depth off, explicit profile), wired into T1.
- **No camera in the URDF:** `grep -i camera src/dagger_description/` returns nothing → there is no `base_footprint → camera` TF, so a bbox bearing cannot be converted into a LiDAR bearing. **Needs a tape measure from the user** (x forward, y left, z above ground, and the aim/pitch) — same situation as the LiDAR height on 2026-09-14. Reference: LiDAR at `base_link` (0.2925, 0, 0.92) = `base_footprint` z 1.12.
- D435 (USB 8086:0b07) enumerated; `librealsense2` 2.58.2 + `realsense2_camera` 4.58.2 built. **D435, not D435i — no IMU.** Depth stays off: it washes out in sunlight and the LiDAR is the range source.
- *Built:* `launch/camera.launch.py` (colour only, explicit profile) + `use_camera` on `drive_bringup.launch.py`. **Measured: 30.00 Hz** (600 frames / 20 s, inter-frame 33.3 ms), 11 % of six cores *including* a subscriber deserialising every frame. Intrinsics fx 606.04, fy 605.56, cx 329.92, cy 259.31, 640×480, `plumb_bob`, frame `d435_color_optical_frame` → **55.7° horizontal FOV (±27.8°)**, i.e. ±1.58 m lateral at 3 m. The follow controller must keep the target inside that or lose it.
- *User (2026-09-25):* the camera is mounted **just below the LiDAR** — so x ≈ 0.2925, y ≈ 0, which puts camera and range source on nearly the same vertical axis and makes the bearing transfer almost exact. Still needed: **how far below** (z) and whether it is pitched down.

**Q6 — ⚠️ Camera published nothing on the expected topic, three separate reasons** ✅ *(2026-09-25, all three found the hard way)*
- *1. Hardcoded namespace.* `realsense_node_factory.cpp:35` constructs the node as `RosNodeBase("camera", "/camera", ...)`, and the driver publishes on **private** topics (`~/color/image_raw`). So the topic is `<namespace>/<node name>/color/image_raw`: namespace unset + name `d435` → `/camera/d435/...`; namespace `d435` + name `d435` → `/d435/d435/...` (this is why stock `rs_launch.py` gives `/camera/camera/...`). **`namespace="/"` + name `d435` → `/d435/color/image_raw`.** Note `camera_name` is a *frame-id* prefix only (`parameters.cpp:83`), not a topic prefix.
- *2. Lazy publishing.* `base_realsense_node.cpp:1169` publishes a frame only `if (0 != image_publisher->get_subscription_count())`, while `camera_info` publishes unconditionally — so "camera_info flows but image_raw does not" is the *normal* idle state, not a fault. Publisher QoS is RELIABLE / TRANSIENT_LOCAL.
- *3. Orphans stole the device.* Killing a `ros2 launch` parent left `realsense2_camera_node` running (H13 again); a second instance then fought the first over USB and the measured rate collapsed to 1.3 Hz with a 33 ms inter-frame gap — i.e. brief 30 Hz bursts, not a slow camera. **`pgrep -x` does NOT find it:** Linux truncates `comm` to 15 chars, so the name is `realsense2_came`. `pgrep -f` matches the calling shell (H-gotcha). Use `ps -eo pid,args | grep` and kill by PID.
- *4. `ros2 topic hz` printed nothing* even when frames were flowing: its stdout is block-buffered into a pipe and `timeout`'s SIGTERM discards the buffer. Measure with a counting subscriber, not `hz | tail`.

**Q7 — `person_detector` node ✅ *built + verified 2026-09-25, `1ac360f`*** — `/d435/color/image_raw` → YOLOX/TensorRT → `vision_msgs/Detection2DArray` on `/follow/detections`. Detection only: no target choice, no range, no motion. Detections carry the **image's** header so the selector gets capture time + camera optical frame. Wired into `follow.launch.py` behind `use_detector` (engine path + `debug_image` args).
- *Rate throttled to 10 Hz on purpose* (LiDAR rate, not the camera's 30) with frames **dropped, not queued** — a backlog would feed the controller a stale bearing.
- ⚠️ *Throttle needed a tolerance:* frames only exist on the camera's 33.3 ms grid, so a strict "has a full period elapsed" test rejects the frame landing a hair early and the rate quantises down a whole frame — 10 Hz came out as **8.55 Hz**. At `0.9 * period` every 3rd frame passes: **measured 9.95 Hz, 100.2 ms median**.
- *Verified end-to-end through ROS* against a known image: 3 people at 0.85 / 0.81 / 0.69, box centres matching the offline run, header preserved, 3 detections on every message of the run. Live camera: 9.85 Hz.

**Q8 — D435 mount measured → URDF camera link ✅ *`ca60c1b`*** — user measured the lens centre at **0.99 m above ground, aimed dead ahead, no pitch, directly below the LiDAR**. `base_joint` lifts `base_link` 0.2 m → joint z **0.79** (the LiDAR's 1.12 is 0.92 by the same arithmetic; camera sits 0.13 m below it). x taken as the LiDAR's 0.2925, y = 0.
- *Of the three, y is what matters:* a lateral error becomes bearing error directly (0.05 m off centre = 1° at 3 m), while an x error is almost purely radial and barely moves the bearing.
- The child link **must** be `d435_link` — the driver derives its frame prefix from `camera_name` (`parameters.cpp:83`) and publishes `d435_link → d435_color_optical_frame` itself; this joint attaches that subtree to the robot.
- *Verified live against the running driver:* `base_footprint→d435_link` (0.2925, 0.0000, **0.9900**) yaw 0.0 · `→d435_color_optical_frame` (0.2925, **0.0147**, 0.9902) yaw −89.8 · `→lidar_link` (0.2925, 0, 1.1200). The 14.7 mm on y is the factory offset from the depth imager to the colour one — which is exactly why the selector transforms through TF instead of assuming the lens is on the centreline.

**Q9 — `target_selector`: bearing from the camera, range from the LiDAR ✅ *`09793d8`*** — `/follow/detections` + `camera_info` + `/scan_safety` → `/follow/target` (`PoseStamped`, base_footprint). Selects and locates only; commands no motion.
- bbox centre + intrinsics → ray → rotated into base_footprint through TF **at the image's own capture stamp**. Scan points are matched as seen **from the camera's real origin**, not the robot's. Points beyond the nearest by more than `cluster_depth` are dropped so the wall behind a person is not mistaken for them.
- **No range → no target.** It will not publish a bearing with a guessed distance. An already-tracked target inside the association gate beats the selection mode, so the rover does not swap people when someone walks past.
- *Round-trip verified with no hardware:* people projected into pixels through the real intrinsics at known positions, then recovered from a synthesised scan. Target at (3.00, +0.40) came back **(3.001, +0.391) — 9 mm error**; a 7 m wall spanning every bearing was correctly rejected in favour of the person at 3 m.

**Q10 — `follow_controller` ✅ *built + offline-verified `56a81de`, NOT yet driven*** — `/follow/target` → `/cmd_vel_follow_raw` → `collision_monitor_follow`.
- ⚠️ **Starts DISABLED and publishes nothing at all until `/follow/enable` goes true.** Launching the follow stack must never be what makes the rover move. While disabled it stays off the wire entirely so `twist_mux` drops the slot rather than seeing a held zero.
- ⚠️ **Never commands reverse** (Q1 decision): no rear sensing, and the monitor zeroes the whole command — reverse included — exactly when it would matter. Too close → slow to a stop and wait.
- *Two rover-specific details:* angular commands below ~0.15 rad/s do not break track stiction (measured actual/commanded 0.12 @ 0.02 rad/s, N-series), so it asks for at least that or nothing rather than winding up the FCU integrator until it lets go; and forward speed is scaled while turning.
- *Offline verification, 8/8 PASS:* silent while disabled · forward when too far · holds station in the deadband · **never reverses at 1.5 m too close** · turns toward a left target · respects `max_linear` 0.4 · stops then goes quiet on target loss · stands down on E-stop.
- *Defaults:* distance 3.0 m ±0.35, max 0.4 m/s (where `CRUISE_SPEED`/`CRUISE_THROTTLE` are calibrated and where the 0.14 m stop distance was measured) / 0.5 rad/s.
- `tools/follow_status.py` records the whole chain, one timestamped line per tick (detections, target range/bearing, requested vs passed command, gate state, scan age) — for the solo protocol where the operator is out at the rover.

**Q11 — ⚠️ The D435 is mounted UPSIDE DOWN — wrong TF *and* near-zero detection** ✅ *found + fixed 2026-09-25, `73451b2`*
- *Symptom:* first live tracking test found a person on **1 tick out of 194 (1 %)**. Detector, mount height and TF had all measured correctly, so nothing pointed at the cause until a raw frame was actually looked at: the scene is only upright after a **180°** rotation. (Claude first read the frame as 90° off; **the user corrected it to 180°**, which the frame confirmed — look at the image, do not reason about it.)
- ⚠️ *The dangerous half was the TRANSFORM, not the detection.* `d435_joint` had `rpy 0 0 0` while the sensor is rolled 180°, so the optical axes were mirrored: **a person on the rover's left would have been computed as on its right and the follow controller would have turned away from them.** Joint now carries `roll = pi`.
- *Confirmed live* by pushing the URDF to `robot_state_publisher` (no T1 restart, so no `mav_frame` reset): `base_footprint→d435_color_optical_frame` RPY went −90.4, 0.2, −89.8 → **+89.6, −0.2, +89.8**, and the colour imager's y offset flipped **+0.0147 → −0.0147** — which is what turning a camera over does to it.
- *The detection half is a model limitation:* YOLOX is **not rotation invariant**, so an upside-down standing person reads as lying down. The detector now rotates frames for **inference only** and maps boxes back to raw pixels, so `camera_info` + the URDF stay the single source of truth for geometry. `image_rotation_deg` is a parameter; `follow.launch.py` sets 180 and both places reference each other.
- *Unit-tested on a flipped reference image:* the no-rotation path finds **1 of 3** people (reproducing the failure exactly), the rotate-and-unrotate path finds **3 of 3 with 0.00 px** corner error vs upright ground truth.

**Q12 — ★ TRACKING VALIDATED ON THE REAL ROVER (2026-09-25, `track4.log`)** ✅ — rover disarmed, controller disabled, person walking 2.2–6.5 m.
| | before Q11 fix | after |
|---|---|---|
| ticks with a person detected | 1 / 194 (1 %) | **110 / 110 (100 %)** in-frame |
| ticks with a LiDAR range lock | 5 (3 %) | **110 / 110 (100 %)** |
| longest continuous dropout | — | **0.0 s** (controller tolerates 0.6 s) |
- *Range stability holding station:* 3.45 m · 5.75 m · **2.24 m ± 0.020 m**. **Open question P-FOLLOW-1 ANSWERED: a standing adult holds a range lock through the 0.30–1.60 m `/scan_safety` band with zero dropouts**, so ground segmentation is NOT a prerequisite for follow-me.
- *`raw` command column stayed empty throughout* — the disabled controller correctly published nothing.
- ⓘ *Consistent bearing bias −1.6° to −2.5°* at all three hold distances (12 cm at 3.4 m). Inside the controller's 5° deadband so it will not steer; watch whether it biases the driven follow. Slight camera yaw or the subject standing off centre — not yet separated.
- 🔴 *Still untested:* the lateral **FOV limit** — bearing only ever reached −16°, so the ±27.8° edge was never probed.

**Q13 — Follow-path gate test (2026-09-25): run 1 partial, run 2 ✅ FULLY VALIDATED**
- *Validated:* **SLOW engaged at 2.48 m** (zone edge 2.60 m) and scaled 0.30 → **0.12 = ×0.4 exactly**, twice, recovering to CLEAR each time the person stepped aside. **`FpApproach=4` vertices throughout** → the Q2 fail-open-and-silent trap is confirmed absent on the follow monitor.
- ⚠️ *NOT validated:* the **STOP zone never fired**. Closest approach was **1.96 m**; the stop polygon only starts at **1.45 m** from `base_footprint` (~0.6 m in front of the bumper). People naturally stop at the slow-zone distance.
- ⚠️ *The tool reported 4/4 PASS anyway* — `classify()` read the **first tick's absent output as a commanded zero**, so STOP was satisfied by startup silence before the monitor had published anything (the polygon columns read `-` on that line, which was the tell). **Fixed `74f505f`:** absent output is `waiting` until output has been seen; the summary now prints the closest approach reached and says how close to walk when nothing entered the zone.
- *Lesson:* a green test line is only worth what its failure mode is worth — this one could pass without the thing under test ever happening.

- ✅ **RUN 2 (13:48), stop zone genuinely entered — every transition landed on its design edge:**

| event | measured | design |
|---|---|---|
| SLOW engaged | **2.50 m** | 2.60 m |
| STOP engaged | between **1.51 and 1.32 m** | 1.45 m |
| closest approach | **1.01 m** | — |
| STOP released | 1.48 m | 1.45 m |
| CLEAR restored | 3.07 m | 2.60 m |

- Slow ratio **0.30 → 0.12 = ×0.4 exactly**; `FpApproach=4` vertices throughout; CLEAR → SLOW → STOP → SLOW → CLEAR with no sticking.
- ⓘ `STOP(no output)` at 13:48:29 is `stop_pub_timeout: 2.0` working as designed — after 2 s of continuous zero the Humble monitor ceases publishing altogether, and `cmd_vel_safety`'s 0.5 s `command_timeout` holds the FCU output at zero from then on.
- **The follow command path now has an independently validated stop zone** (the Nav2 monitor's 09-17 validation was the same geometry but a different node instance).

**Q14 — ⚠️ `preflight.py` reported SET_MESSAGE_INTERVAL FAIL on commands that SUCCEEDED** ✅ *fixed 2026-09-25, `b97f79b`*
- *Symptom:* after a rover restart, preflight failed all five `SET_MESSAGE_INTERVAL` calls and blocked the driven test — while **every stream they configure was measuring at exactly its target rate**.
- *Cause:* `Preflight.call()` used a **3.0 s** timeout. Calling `/mavros/cmd/command` directly returned **`success=True result=0` in 4.20 s and 3.08 s** — CommandLong round-trips over the 57600-baud FCU link normally take 3–4 s and longer while the FCU is busy (e.g. just after a reboot). The calls were timing out, not failing.
- *Fix:* CommandLong timeout 3 → **8 s with one retry**. Verified `PREFLIGHT PASS`, all five OK.
- *Why it matters beyond the noise:* preflight is the gate that catches `mav_frame` left at `LOCAL_NED` (N1), which drives the rover the **wrong way**. A gate that cries wolf is one you learn to skip.
- ⓘ *Also seen on that bring-up, both transient and self-clearing:* `gps_yaw=UNAVAILABLE` for one run (walled-lane multipath while the GNSS re-acquired after the reboot; back to 47–52° with `hdg_acc 0.00` on later runs) and `/mavros/state` at 0 Hz / `pubs=0` for one run (the known cosmetic discovery quirk). **Root cause of the whole cluster: the user had restarted the rover**, which also took the four terminals down mid-test.

**Q15 — FCU params after the rover restart (2026-09-25)** ✅ — all persistent and intact: `CRUISE_SPEED 0.4`, `CRUISE_THROTTLE 45` (the N8 braking fix), `ATC_STR_RAT P 0.3 I 0.2 IMAX 0.2 FF 0.8`, `SERVO1_MAX 1900` (P5 still deferred), `FS_THR_ENABLE 1` (P4 still open).
- 🔴 *One deviation from the recorded configuration:* **`WP_SPEED` now reads 0.4, not 2.0.** It was set to 2.0 explicitly (N8) so AUTO missions would not inherit the cruise calibration. Harmless for follow-me, which uses no AUTO, but it will silently slow any Mission Planner AUTO run — which is the **P6 benchmark**. Restore before the P6 comparison: `ros2 param set /mavros/param WP_SPEED 2.0` (undo: 0.4).

**Q16 — ★★★ FIRST DRIVEN FOLLOW-ME — THE ROVER FOLLOWED A PERSON (2026-09-25 15:36)** ✅
Full chain end to end: D435 → YOLOX/TensorRT → bearing + LiDAR range → follow controller → follow collision monitor → twist_mux → cmd_vel_safety → FCU. **User confirms the rover moved as expected.**
| | |
|---|---|
| target lock | 156 / 158 ticks |
| range held | 3.3–3.4 m (set 3.0 ± 0.35) |
| max speed | 0.357 m/s (limit 0.4) |
| reverse | never — held 0 when the person closed to 2.20 m, resumed as they moved away |
| turn sign | correct both ways (+7→+15° gave w +0.150→+0.233; −12→−20° gave w −0.186→−0.314) |
| monitor activity | approach polygon scaled the command ×0.71–×0.96 live |
- The `min_effective_angular` stiction floor is visible as w snapping to exactly ±0.150.
- ⚠️ **NO BAG WAS RECORDED.** The earlier no-op run had already created `pf_follow_walk_1`; `ros2 bag record` refuses an existing folder and exited, and the script drove anyway. **The first drive's odometry is unrecoverable.** Fixed `492d92b`: bag names carry a per-run timestamp and a recorder that fails aborts the run **before** follow is enabled.
- ⚠️ *Two earlier attempts were no-ops from `ros2 topic pub --once`*, which creates a publisher, fires and tears down, so a reliable subscriber misses it before discovery completes (it printed "publisher's context is invalid" — the tell). Measured: `--once` did not arrive; `-t 10 -r 5` arrived instantly. Both enable and disable now publish repeatedly — a dropped **disable** would leave the rover following.
- ⓘ *`follow_status` display bug fixed:* the gate column tested "no output" before "nothing requested", so a rover **holding station** (commanding zero, monitor correctly silent after `stop_pub_timeout`) was labelled STOP. Nothing was blocked.

**Q17 — Known limits of target selection (asked by the user 2026-09-25)** ⓘ *by design, not yet addressed*
- **Multiple people:** every detection over `min_score` 0.5 becomes a candidate. An already-tracked target wins if its bearing is within `association_gate_deg` **20°** of last frame; otherwise `selection_mode` decides (`centred` = smallest |bearing|, default; or `nearest`). **The association is BEARING-ONLY — range is computed but not used for it, and there is no appearance model or re-identification.** So: someone crossing at a similar bearing can steal the lock; after an occlusion it may resume on the wrong person; and it re-decides every frame with no indication that the target changed.
- **Person leaves the frame:** detector finds nothing → selector publishes nothing → controller `target_timeout` 0.6 s → zeros for 1 s → goes quiet → twist_mux drops the slot. **The rover STOPS. It does not search, turn to look, or extrapolate.** After `lost_timeout` 1.5 s the selector clears `last_bearing`, so re-acquisition is a fresh `selection_mode` decision — which may pick a different person.
- *FOV arithmetic:* 55.7° (±27.8°); max turn 0.5 rad/s ≈ 28.6°/s. At 3 m the target would have to move sideways at ~1.5 m/s to out-turn the rover, so straight-line following is comfortable — **the exposure is corners and occlusions, not walking pace.** Following further out is angularly more forgiving than following close.

**Q18 — Target selection hardened after the first drive (2026-09-25, `ed20c41`)** ✅ *built + bench-verified, not yet driven*
- **Association now matches RANGE as well as bearing** (`association_range_gate` 1.5 m). Bearing alone let a crosser at a similar angle but a very different distance steal the lock — the range was already computed and simply unused.
- **Swaps are no longer silent:** the selector keeps a track id, increments it whenever an acquisition is not a continuation, publishes `/follow/track_id` and logs what changed (old vs new bearing/range, number of people in frame).
- **Loss now searches before giving up:** turns toward the last known bearing for `search_timeout` 3 s. Two constraints, because the rover is moving with nothing in view — **ROTATION ONLY, never forward** (`cmd = Twist()`, only `angular.z` set), and only if `|last_bearing| >= search_min_bearing_deg` 8°, since vanishing near dead ahead means occlusion or out of range and turning would point the camera **away** from where the person still is.
- **If the search fails, follow DISABLES itself** — the rover never picks a new target on its own; re-enable is explicit (user's choice).
- ⚠️ *Bug found while testing:* auto-disable fired on the **first tick after enabling**, before any target arrived — it read "no target" as a loss. The gate guarantees a target before enabling so it never bit in the field, but a detection blinking at the moment of enable would have switched follow straight back off. Auto-disable now requires having actually held a target.
- *Verified 7/7 in an ISOLATED DDS DOMAIN* (the real stack was armed in GUIDED, and a second `follow_controller` on domain 0 would have driven the rover): searches after an off-axis loss · search is rotation only (**v exactly 0.000 for the whole window, w +0.250**) · turns toward the target · stops when the search expires · does not resume on its own · resumes on explicit re-enable · no search when lost dead ahead.
- ⓘ *Test-methodology note:* the first run of this test showed "search is not rotation only" — the sampling window included the 0.6 s `target_timeout` during which the controller is still legitimately **following** the last target. Measure a search window only after that expires.

**Q19 — ★ PHYSICAL MOTION CONFIRMED FROM ODOMETRY (bag `pf_follow_walk_1_161016`, 2026-09-25 16:10)** ✅
The 15:36 drive had no bag (Q16), so this is the first follow run with independent evidence rather than commands alone.
| | |
|---|---|
| path length travelled | **9.88 m** |
| net displacement | 5.38 m |
| FCU body velocity > 0.05 m/s | 584 / 1260 samples |
| target lock | 40 / 41 ticks, range 4.72 → 2.98 m |
| max commanded | 0.400 m/s (the configured ceiling) |
- ⓘ **Peak ACTUAL speed 0.560 m/s against 0.400 commanded.** Transient, not a runaway: median while moving 0.243 m/s, p95 0.450, only **5.1 %** of moving time above 0.45; and while the setpoint sat at its 0.4 ceiling the actual **mean was 0.330** (slightly under) with p95 0.489. Normal closed-loop overshoot on acceleration — but it means stopping distance is somewhat longer than the **0.14 m measured at a steady 0.4 m/s** (N8). Worth re-measuring stop distance from a follow-me acceleration transient before raising `max_linear`.

**Q20 — The recorder guard aborted healthy runs** ✅ *fixed `982df61`*
- The Q16 fix (abort if no bag) used a flat 3 s wait. `ros2 bag record` subscribes to all **17** topics before creating its output folder, which takes longer, so two runs were aborted on recorders that were starting normally — and the abort path killed the status logger but **not** the recorder, leaving two orphans writing to disk.
- Now polls up to 25 s, gives up early if the recorder process has actually died, kills **both** children on failure, and prints the bag path once recording is confirmed.
- *Pattern worth remembering:* a guard added to catch a real failure (Q16) became a new failure of its own by being stricter than the thing it guards. Both times the run was lost.

**Q21 — ⚠️ The loss-search turned the rover barely at all (2026-09-25 16:30, bag `pf_follow_walk_2_163018`)** ✅ *rate raised `70e3c91`, not yet re-driven*
- *User report:* walked left, then sharply right out of frame; "rover didn't rotate right fully".
- **The direction logic was correct.** From `/rosout`: lost at **+23.6°** → turned toward it → **reacquired within 2 s** at +24.3° (track 22); then lost at **−23.7°** → turned that way too → not recovered → **FOLLOW DISABLED** at 16:30:49, exactly as designed.
- ⚠️ **What failed was the AMOUNT of turn:**

| search | commanded | actually turned | ratio |
|---|---|---|---|
| 1 (left, reacquired) | ~53° | **+23.4°** | 44 % |
| 2 (right, gave up) | ~45° | **−7.6°** | **17 %** |

- *Cause:* **N6 track stiction in a new place.** Search 2 began from exactly zero linear velocity — the worst case for a 350 kg tracked hull — and managed **2.6 °/s** against 0.25 rad/s commanded; search 1 still had 0.04 m/s of residual motion helping it. Below ~0.3 rad/s the tracks do not execute what they are told.
- *Fix:* `search_angular` 0.25 → **0.45 rad/s**, `search_timeout` 3 → **4 s**, so a search can sweep the FOV rather than nudge.
- ⓘ *Limit to keep in mind regardless:* in that run the person reappeared **12 s after the search ended**. A search recovers a corner or a brief occlusion — not somebody who walked away and came back.

**Q22 — The instrumentation failed while proving Q21** ✅ *fixed `70e3c91`*
- `follow_status` died mid-run with **ZeroDivisionError**, exactly when the rover first turned in place: a search is rotation-only and the gate column divided by `raw.linear.x`. It took the log out **50 s before the end**, losing the record of the very event it had been extended to capture. The ratio is now taken on whichever axis was actually requested.
- `pf_record.sh` captured **none of the follow topics**, so the whole sequence had to be reconstructed from `/rosout`. It now records `/follow/target`, `/follow/track_id`, `/follow/detections`, `/follow/enable`, `/cmd_vel_follow_raw`, `/cmd_vel_follow`.
- *Pattern (third time today):* the tooling around the test failed more often than the rover did — Q16 no bag, Q20 guard aborting healthy runs, Q22 log crash. **Instrument the new behaviour before driving it, not after.**

**Q23 — ★ Search rate fix validated on the rover (2026-09-25 16:41, bag `pf_follow_walk_2_164111`)** ✅
`search_angular` 0.25 → 0.45 rad/s turned a nudge into a real sweep:

| search | old (0.25 rad/s) | **new (0.45 rad/s)** |
|---|---|---|
| 1 | −7.6° (17 %) | **−43.6° (64 %)** |
| 2 | +23.4° (44 %) | **+69.7° (70 %)** |

- ~17 °/s sustained, i.e. **70° of sweep in 4 s** — comfortably past the ±27.8° half-FOV. **Search 1 reacquired the target in 2 s** and following resumed.
- ⚠️ *Boundary race:* search 2 expired at **16:41:48** and the selector reacquired at **16:41:48** — the same second. The rover **disabled itself one tick short of succeeding**, having already turned far enough to see the person. `search_timeout` 4 → **6 s** (`b3d4347`) → ~100° of sweep. Not free: that is how long the rover may rotate on the spot with nothing in view, so it stays a parameter.
- ⓘ The gain ratio also confirms N6 from the other direction: at 0.25 rad/s commanded the tracks delivered 17–44 %, at 0.45 rad/s they deliver 64–70 %. **Commanding above ~0.4 rad/s is what makes an in-place turn actually happen on this rover.**

**Q24 — 🟡 0.82 m of travel after a zero command — CAUSE UNRESOLVED, measurement was not interpretable (2026-09-25, bag `pf_follow_walk_2_164710`)**
- *Measured:* setpoint stepped from a sustained **0.400 → 0.000** (angular 0.000 throughout, zeros flowing at 20 Hz) and the rover **travelled 0.82 m over 2.60 s**.
- *Not an estimator artifact (P10) — the tracks were genuinely powered:*

| t−t0 | setpoint | body vel | PWM L/R (neutral 1500) |
|---|---|---|---|
| 0.0 | 0.000 | 0.358 | 1650/1707 |
| 0.8 | 0.000 | **0.450** | 1717/1634 |
| 1.5 | 0.000 | 0.288 | 1688/1598 |
| 2.5 | 0.000 | 0.137 | 1554/1564 |
| 3.0 | 0.000 | −0.027 | 1485/1515 |

- **Throttle held flat at ~44 % for 0.8 s and the rover ACCELERATED to 0.450 m/s after being commanded zero**, then decayed over ~2.2 s. Hold-then-decay is the signature of a wound-up integrator, i.e. the **N8 mechanism**, which the CRUISE calibration was believed to have fixed. N8's 0.14 m / 0.70 s came from a Nav2 goal cancel; this is a follow-controller step to zero from a saturated 0.4 m/s.
- *FCU state:* `ATC_DECEL_MAX = 0` (falls back to `ATC_ACCEL_MAX` 1.0 m/s², which alone would stop it in 0.4 s) · **`ATC_SPEED_IMAX = 1.0`** with `ATC_SPEED_I = 0.2` — the speed integrator is allowed to hold **up to full throttle** and unwinds slowly. `ATC_BRAKE = 1`, `MOT_SLEWRATE = 100 %/s` (would allow 0.45 s), `CRUISE 0.4/45` confirmed.
- ⚠️ **Why this matters:** the collision monitor's stop polygon reaches **1.45 m from `base_footprint` = 0.6 m in front of the bumper**, sized on the belief that braking takes ~0.3–0.4 m. **0.82 m of roll exceeds it.**
- ⓘ *What still protects the normal case:* the layered design. The slow zone engages at 2.6 m and scales to ×0.4, so an approach reaches the stop zone at ~0.16 m/s, not 0.4 — which is why the N7 live tests rolled only 0.01–0.02 m. **The exposed case is a person appearing SUDDENLY inside the slow zone**, where the rover steps from 0.4 m/s to stop with 0.6 m of margin and needs 0.8 m.
- ⚠️ **CORRECTION — this may not be an FCU fault at all.** Claude concluded "N8 is not fixed" after checking that the mode stayed `GUIDED` throughout, reasoning that ArduPilot ignores RC sticks in GUIDED. **The user, who was operating the transmitter, reports that this rover DOES move on RC throttle while in GUIDED (forward/back only, no steering)** — and the cut at 16:48:29 was the script's end-of-run disable, i.e. exactly when they had picked up the RC. The observation from the operator supersedes the general-behaviour assumption.
- ⚠️ **The data could not settle it, because `/mavros/rc/in` was never recorded — or even published.** `/mavros/rc/out` is the *servo output* and looks identical whether the command came from ArduPilot or from the pilot's sticks. mavros only publishes `rc/in` if the **`RC_CHANNELS` (msg 65)** stream is enabled, and preflight did not request it. **Fixed:** msg 65 at 10 Hz added to `preflight.py` STREAMS with a rate gate, and `/mavros/rc/in` added to `pf_record.sh`. Verified live: 10.0 Hz, PREFLIGHT PASS.
- *Also n = 1:* this was the only abrupt setpoint step in any follow bag, because normal follow deceleration rides the controller's acceleration ramp and never steps.
- *Next:* repeat a follow run **without touching the RC during it or for 10 s after**, then re-measure. If 0.82 m reproduces with `rc/in` flat, it is the FCU and the candidates are `ATC_DECEL_MAX` 0 → 1.5–2.0 and `ATC_SPEED_IMAX` 1.0 → ~0.3 (user's domain). **Hold `max_linear` at 0.4 until then** — not because the finding is proven, but because it is unresolved.
- ⓘ *Separate observation from the same trace:* the track outputs were **oscillating** (1706/1640 → 1649/1705 → 1710/1637, swapping which side leads) while following — the steering controller hunting. Not investigated.

**Q25 — Clean-RC run (2026-09-25 17:17, bag `pf_follow_walk_2_171710`): instrumentation works, stop question STILL OPEN** 🟡
- ✅ **`/mavros/rc/in` recorded, 695 samples, and the sticks were dead flat** (`1495/1495` on every channel, spread 0, including the last 15 s). So this run is uncontaminated — and the Q24 ambiguity is now resolvable in principle.
- ⚠️ *But the event did not recur, because there was nothing to measure:* at both target losses the rover was **already stationary and pivoting**, not driving — body velocity −0.03 and −0.10 m/s, travelling **0.06 m and 0.01 m** in the 3 s after the linear command went to zero. Largest setpoint drop in the whole bag was 0.015 (one ramp step); only 198/1676 samples were non-zero.
- *To reproduce Q24 the rover must be at ~0.4 m/s at the instant the command zeroes.* Protocol: walk away **briskly** so it holds the 0.4 cap for a few seconds, then **leave the frame abruptly** — the loss cuts linear to zero instantly at full speed. Safe: the person is 3+ m ahead and moving away.
- ⓘ *Run shape:* enabled 17:17:24, lost at +21.1°, reacquired, lost at −19.2°, **disabled 17:17:50** — two losses in 26 s. At 3 m with a ±27.8° FOV only ~1.6 m of sideways movement leaves the frame, so brisk direction changes drop the lock easily. **This is the FOV limit, not a tracking fault**, and it is the strongest argument for the LiDAR-cluster tracking the user proposed (the LiDAR is 360°).
- ⓘ The 6 s search was active and pivoting hard (`rc/out` differential up to ±207 µs).

**Q26 — Stop test 17:32 (`pf_stop_test_173211`): the rover moved, the bag's FCU topics missed it** 🟡
- *Claude first reported "the rover never moved"* (actual 0.021 m/s, `rc/out` neutral). **The user corrected it: it moved.** Every `/mavros/*` topic in the bag — `rc/out`, `rc/in`, `velocity_body`, `state`, `imu` — **starts at 17:32:45, six seconds after the event** (17:32:26–39), with zero samples inside it, while `/odometry/filtered` recorded throughout (3.59 m path). The recorder took ~30 s to discover mavros's publishers (the known `/mavros/state` discovery lag).
- ⚠️ *Why the analysis said "never moved":* the lookup helper used `max(0, bisect(...)-1)`, which for a time **before the first sample silently returns the first sample** — so it printed 17:32:45 values as if they had held for the whole event, which is what made them look frozen. **Never clamp a time lookup to index 0; return None before the first sample.**
- *Fix (`follow_solo.sh`):* wait for the recorder's own `Subscribed to topic` lines for `rc/out`, `rc/in`, `velocity_body` and the setpoint topic (up to 45 s) and refuse to enable follow without them.
- *Measured from odometry alone:* target lost dead ahead → immediate disable → abrupt zero, but the setpoint had only ramped for ~0.5 s, so **speed at the cut was 0.06 m/s**. The rover **kept accelerating for 0.4 s after the zero**, peaked at 0.24 m/s, and **rolled 0.19 m, at rest in 1.12 s**. → **~0.4 s command-to-motion lag**: a stop from 0.4 m/s pays ~0.16 m before braking even begins. Q24 (the 0.4 m/s stop) is **still unmeasured**.

**Q27 — ★ LiDAR hold: the 360° LiDAR keeps the person the camera loses (2026-09-26, `1c8f06c`)** ✅ *built + simulation-verified, NOT yet driven*
- *Why:* the camera's 55.7° FOV was the binding limit — at 3 m ~1.6 m of sideways movement leaves the frame; Q25 lost the target twice in 26 s that way.
- *Design:* **camera CONFIRMS, LiDAR KEEPS.** Confirmed target tracked in the **odom frame** (position + smoothed velocity); with no camera match, carried on the scan cluster nearest the predicted position for up to `lidar_only_timeout` **5 s**. `/follow/target_source` = camera|lidar.
- *Guards (the LiDAR cannot tell a person from a post):* gate 0.6 m + walking allowance · `max_cluster_width` 0.9 m + span check · the 5 s limit.
- ⚠️ **Bug found by the sim, pre-dating this work: the lock hopped to a bystander with NO id change.** Association was nearest-in-bearing then a 20°/1.5 m gate; with two people ~1 m apart, as the tracked person walked across, the bystander became nearest-in-bearing and passed the gate — the rover **followed the bystander at 0.36 m/s**. Now associated by **predicted odom position** (bearing gate only as a fallback without TF).
- ⚠️ **Second gap: a seamless handover to a new person.** When a hold expires with someone else in frame, the selector publishes them with no gap, so the controller never saw a loss. **The controller now latches the track id at enable and DISABLES if it changes** (consistent with the user's explicit-re-enable choice). Selector publishes the id before the target.
- *Controller:* beyond **35°** bearing → **turn in place (v = 0)**; the old 60 % turn-speed rule would have driven forward past a person beside the rover.
- *Also:* camera `max_range` 8 → **15 m**; `/scan_safety` `range_max` 10 → **20 m** (needs T2 restart).
- *Tests now in `tools/`* (scratchpad copies were lost overnight), pinned to **ROS_DOMAIN_ID 42** so they cannot reach an armed rover:
  - `test_follow_lidar.py main` **4/4**: holds to +56° and −54°, turns in place beyond 35°, no hop to a bystander in view, and on expiry the switch disables follow.
  - `test_follow_lidar.py wall` **3/3**: person 0.35 m from a wall merges into a 1.14 m return → refused, nothing published, never drives. *(Limitation: a person flat against a wall out of camera view is lost, deliberately.)*
  - `test_follow_controller.py` **11/11**, incl. the track-id disable.

**Q28 — Offline work 2026-09-26: three open items closed** ✅
- **"Steering oscillation" was not one** (`7a79932`). From a 4 s window it looked like hunting; over 21.9 s of steady driving at zero yaw command the L−R PWM changed sign 3 times (period ~15 s) with gyro z std 0.048 rad/s. The persistent **−43 µs mean** is the FCU steering integrator compensating the **P5 track imbalance** — fresh evidence for `SERVO1_MAX 1830`. The real controller effect was on/off chatter at the 5° deadband edge (median bearing 4.3°) with the 0.15 rad/s stiction floor → **hysteresis: turn above 5°, stop below 3°**. Test: 4.5/5.5° dither gives 1 toggle. Controller suite **12/12**.
- **Recorder "subscribed ≠ receiving"** (`d0876f4`): new `tools/fcu_flowing.py` counts rc/out, rc/in, velocity_body and exits non-zero below minimum rates; `follow_solo.sh` refuses to enable without it, then gives the recorder 3 s. *(Draft bug caught: grepping for "FLOWING" matched "NOT FLOWING" — now uses the exit code; verified exit 1 with the stack down.)*
- **P10 ZUPT** (`d0c42f0`): `fcu_odom_relay` zeroes linear velocity once both tracks have held within 25 µs of neutral PWM for 1.0 s; stale rc/out switches the gate off. `tools/test_odom_zupt.py` replays the original P10 stop through the real relay: **false slide at rest 0.240 → 0.000 m; real coasting 0.077 vs 0.078 m (untouched)**. Needs T2 restart.
- ⓘ *Test-design lesson, twice in one day:* both the lidar and the ZUPT tests first "failed" on windows that mixed two phases (following vs searching; coasting vs estimator slide). Define the window from the physics before trusting a verdict.

**Q29 — ★ Ground segmentation (P2) built and wired in (2026-09-26, `642c846`)** ✅ *built, NOT yet validated live*
- **Patchwork++** vendored at `src/patchwork-plusplus` (gitignored, pinned **3e6903a**, reproduce with `tools/fetch_patchworkpp.sh`; build only the `ros/` wrapper, `cpp/` + `python/` COLCON_IGNOREd, single worker). **15.6 ms median, 17 % of one core** on the Orin.
- ⚠️ The node **does not transform** — it only stamps output with `base_frame` → must be the INPUT frame (`rslidar`) or every point is silently mislabelled. `min_range 1.0` also drops mast self-returns.
- `ground_seg:=true` (default): costmap `/scan` = p2l on `/patchworkpp/nonground`, **floor 0.35 → 0.12 m**.
- **Old band kept as a second costmap source** (`/scan_band`, raw 0.35–0.75, always published): replay showed seg adds ~5 low-obstacle rays/scan but loses ~1.7 the band sees (none < 1.5 m, median 6.8 m) — with both sources the costmap can only *gain* obstacles.
- ⚠️ **Costmap staleness was never checked**: `expected_update_rate` defaulted to 0.0 = never stale, so a dead `/scan` (P9 wedge, or Patchwork++ dying) left Nav2 driving blind. Now **1.0 s on both sources**; Humble's controller waits on `isCurrent()` before computing velocity (`controller_server.cpp:402`) → **fails closed**. Also covers P9.
- `scan_watchdog` now watches p2l's actual input cloud.
- ⓘ *Offline replay caveats:* the replay could not show the slope benefit (site was flat; with the correct 1.12 m height a bare 0.12 m floor already clears flat ground), and the first "what does seg hide" comparison paired scans by index across independent streams — numbers moved between identical runs; fixed to pair by cloud timestamp. **Per the user, validation is live** (old bags not used further).
- *Live test:* 15–25 cm object 3–5 m ahead must appear in `/scan` + costmap while flat ground does not; then a detour around it.

**Q30 — ⚠️ LIVE: low obstacles were ERASED from the costmap as the rover closed in → separate low-obstacle layer (`f0aed29`)**
- *Measured (`tools/low_obstacle_approach.py`, RC drive toward a box):* seen and marked lethal at **4.36 m**; LiDAR lost it at ~3.5 m (beams pass over); **at 3.23 m it was ERASED from both costmaps** — empty rays cleared its cells — with the rover still ~2.4 m short. Collision monitor is blind that low too.
- *Placement lesson:* Claude first told the user to put the box 2.5–3 m from the bumper, **inside the LiDAR's blind band** (raw cloud had ZERO points 1.5–4 m ahead). Compute the beam geometry before placing a test object. For a 20 cm object, windows of clean visibility are ~3.7–4.0, 4.3–4.6, 5.0–5.4 m from the rover centre (−15/−13/−11° beams); between them the hit is < 12.5 cm and Patchwork++ calls it ground.
- *Fix (operator chose it):* `low_obstacle_layer` in both costmaps, fed by `/scan_low` = low-ONLY returns (12–35 cm with nothing 40–160 cm at the same bearing/range, via `p2l_low_all` + `p2l_tall` + `low_scan_filter`), **ray-clears only beyond 4 m** (`raytrace_min_range`). A separate layer, so the normal layer's rays cannot erase it; people/walls excluded so they don't leave persistent marks. Filter unit test 5/5. *Live approach re-test still pending.*
- ⚠️ *Open:* a ~1.7 m arc 50–70° left at ~5 m is classified entirely non-ground (median height 0.06 m, spread −0.33..+0.59) — either a real drop/kerb or rough ground the plane fit fails on. **If false, the new layer would REMEMBER it.** Needs the operator's eyes.

**Q31 — ★ Rotated ghost copies in the global costmap: odom heading followed GNSS jitter (`3d49001`)** ✅ *verified live*
- *Symptom (operator):* global costmap shows "2–3 maps in rotated manner", worst after every T2 start.
- *Cause 1 — jitter:* at rest over 30 s the gyro integrated **+0.01°** while fused odom yaw spanned **1.70°** in steps up to **0.95°**, copying the GNSS input (span 1.71°). Each step rotated the odom frame the costmaps live in; old marks landed just behind the new ones and could never be ray-cleared (Claude's audit first misread them as "684 occluded cells").
- *Cause 2 — startup snap:* `ekf_local` has no initial heading and publishes TF at once → costmaps mark the first scans at yaw 0 → first GNSS heading snaps the frame ~48° round.
- *Fix:* `gps_heading_imu` is now a **complementary filter** — gyro integrated continuously, GNSS corrects only the gyro-to-true offset with **tau 20 s**; outliers > 10° ignored unless they persist 5 s (then re-seed); seeded from 10 readings. **Once locked, and on any re-seed, it clears both costmaps once** (the operator's request: clear/start mapping only when everything is up). Output sd 1.0 → 0.5°.
- *Verified:* offline — locks in 1.9 s, steady span **0.041°**, 20° spike rejected, 30° persistent change re-seeds. **Live at rest — fused-minus-gyro span 1.70 → 0.48°, largest step 0.95 → 0.016° (60×).** Operator confirms maps now show correctly.
- ⓘ Foxglove showed "missing transforms" after the T2 restart while the ROS TF tree was complete — client-side stale state; reconnecting fixed it.
- ⓘ The FCU's own position estimate crept **1.6 m in 45 s at rest** in the walled lane (GPS walk). Fused odom doesn't inherit it.

**Q32 — Low-obstacle memory SHELVED by the operator (`37c409a`)** 🟡 *future work*
- On the rover, **a person who walked past left a blob that stayed** — the low-only filter (low return with nothing tall at the same bearing/range) did not keep people out, so the remembering layer reproduced the stale-costmap problem it was meant to avoid. Operator: skip for now, mark for future.
- `low_obstacle_layer` → `enabled: false` in both costmaps; its three feeder nodes run only with `low_obstacle_memory:=true`. Code kept. Disabled live first, costmaps cleared → 105 lethal, all live, 0 stale.
- **Consequence:** ground segmentation still SEES low obstacles from ~4–6 m (it found a box the old band never would), but they are again **forgotten inside ~3.3 m**. The planner usually commits to a detour while the object is visible; nothing guarantees it.
- *Ideas for when it comes back:* the tall-return test is per-bearing, so a moving person's legs can register at a bearing where the torso return lands a ray away — widen the tall check to ±2 rays and require a low return to persist N scans at the same odom cell before it enters memory (people move, boxes don't).
- **New gap logged — negative obstacles:** the operator confirmed the lane's left side is a real **drop to lower ground**. It was only marked because something at the lip sat 12–35 cm up; the lower ground itself falls under the 0.12 m floor. A clean drop-off with nothing at its edge would not be marked at all. Needs a drop detector (e.g. flag cells where non-ground/ground points sit well below the local plane).

**Q33 — ★ LiDAR HOLD VALIDATED ON THE ROVER (2026-09-26 13:04, bag `pf_lidar_hold_1_130411`)** ✅
Operator walked arcs out of the camera's view on both sides; run executed by the solo script, analysed from the bag afterwards (the operator cannot watch the laptop while at the rover).
- Enabled 13:04:49; rover crept 4.0 → 3.25 m and stopped (the visible "follow is on" cue: stand ~4 m out).
- **Camera lost the target 5 times; the LiDAR held it every time and the camera reconfirmed each time. One track id (47) all run — no switches.**
- **Turn in place as designed:** at −37.7°, +52°, +41°, −55.9° the command was **v 0.00, w ±0.50**; forward drive resumed within ~20°. Never reversed (held 0 at 1.41 m). RC sticks flat throughout.
- ⚠️ *One hold expired:* 13:05:14–19 the operator reached +52° faster than the rover could pivot (~17 °/s actual vs 0.5 rad/s commanded — N6 stiction), the 5 s LiDAR-only limit ran out, the controller's search took over and the camera reacquired 1 s later, same track id. Proposal: `lidar_only_timeout` 5 → 8 s (awaiting operator).
- *Tooling:* `/follow/target_source` was not in `pf_record.sh` — camera-vs-LiDAR reconstructed from log messages. Added.

**Q34 — 🔴⚠️ Q24 CONFIRMED: 0.85 m roll after a zero command, and it is the FCU (2026-09-26 14:10, bag `pf_stop_test_140900`)**
- Clean run: **RC sticks flat (spread 0)**, gate PASS → enable in 1 s, cruise ~0.35 m/s, script cut to zero mid-cruise.
- After setpoint 0.000 the tracks held **~1710/1732 (≈55 % throttle) for ~1.0–1.5 s**; FCU speed rose to 0.370 at +1.0 s; neutral only at +3.0 s. **ROLLED 0.854 m, rest in 2.68 s.** Reproduces 09-25's 0.82 m (Q24) — that one was NOT the RC.
- ⚠️ Collision monitor stop zone gives **0.6 m** ahead of the bumper. Normal approach still protected (slow zone cuts to ~0.16 m/s first); **a sudden close-range step-in is not.**
- *Candidates (operator's domain):* `ATC_SPEED_IMAX 1.0 → 0.3` (undo 1.0), `ATC_DECEL_MAX 0 → 2.0` (undo 0). Re-test with the same `DURATION_S=15` run. ROS-side stopgap offered: stop zone +0.5 m. **`max_linear` stays 0.4.**
- **Retest 14:17 (`pf_stop_test_141700`), operator set `ATC_SPEED_IMAX 0.3`, `ATC_DECEL_MAX 2.0` (read back to confirm):** RC flat, cruise 0.340 m/s (vs 0.349 — no loss of speed, peak 0.535) → **rolled 0.577 m, rest 2.34 s** (was 0.854 m / 2.68 s). Throttle 1 s after the cut ~25 % (was ~55 %). **Still only just inside the 0.6 m margin.**
- *Mechanism now visible:* at 0.35 m/s the rover needs ~55 % throttle but `CRUISE_THROTTLE 45 / CRUISE_SPEED 0.4` feed-forward supplies ~39 %; the integrator carries the gap and keeps pushing after a stop, unwinding slowly. Next (one change at a time): `ATC_SPEED_IMAX 0.3 → 0.1` (undo 0.3); then, if needed, recalibrate `CRUISE_THROTTLE` so the integrator has little to hold.
- ⓘ Follow **disable took ~3 s** (printed 14:10:07, cut 14:10:10) — `ros2 topic pub` start-up. Scripted stops only; the **RC remains the E-stop**.
- ⓘ Tooling that fed this: gate-last reorder (`1f86198`, gate→enable 33 s → 1 s); `fcu_flowing` waits for first message (`306c490`, discovery ~6 s); retry + diagnostics (`80c5aaa`). **A false "reproduction" of a telemetry failure was actually the whole stack having been shut down — check the stack is up before diagnosing.**

**Q35 — IMAX 0.1 retest + 🔴 THE ORIN IS CPU-SATURATED AT IDLE (2026-09-26 14:44–14:53)**
- *Stop test, `ATC_SPEED_IMAX 0.1` (read back):* measured as **peak forward travel along the heading** (net displacement was wrong — P10 estimator lag carried odom 0.17 m back after the stop): **IMAX 1.0 → 0.860 m, 0.3 → 0.576 m, 0.1 → 0.517 m** (the last from a faster 0.352 m/s). Big gain from the first step, then diminishing: a **~1 s near-full throttle hold after the zero command persists at every IMAX setting.** Cruise not hurt (mean 0.385 m/s).
- *5 zero-setpoint glitches mid-run:* none from Nav2's path, none from the scan-freshness gate (/scan_safety max gap 0.15 s). **The follow collision monitor's OUTPUT fell to 10.8 Hz with gaps to 0.6 s** while its input was a steady 20 Hz; each glitch is 0.51–0.53 s after its last output → `twist_mux` 0.5 s timeout → one zero. Same stack instance all afternoon: **20.0 Hz (14:09) → 17.9 (14:17) → 10.8 (14:44)**.
- **Cause: the Orin is saturated with nothing driving — ~495 % of 600 %, load average 16–18, all six cores 80–91 %** (1.51 GHz, 64 °C, not thermal). Stable over minutes, not currently growing. Top: **mavros_node 102 %** (single process; main thread 34 % + five executor threads ~10 % — default ~40 plugins, no GCS forwarding), collision_monitor_follow 34 % (idle!), bt_navigator 33 % (idle!), person_detector 21, target_selector 20, gps_map_odom 19, planner/controller/ekf/3×p2l/patchworkpp 15–18 each, **both lifecycle managers 10–13 % idle**. `/tf` only 90/s, `/rosout` 0.4/s — not a shared-stream storm.
- ⚠️ *Why it matters:* the follow **safety gate is starving** (fails safe via twist_mux, but a gate that drops half its messages is not a gate to rely on); and with mavros saturated, **the zero command may reach the FCU late** — which would produce exactly the ~1 s throttle hold that no FCU parameter removed. *Unverified:* the FCU's own target echo (`/mavros/setpoint_raw/target_local`, MAVLink 85) is not streamed, so command→FCU latency cannot be measured yet.
- *Plan (performance pass):* (1) stream msg 85 to measure command→FCU latency directly; (2) mavros `plugin_denylist` for unused plugins; (3) why idle Nav2 nodes and lifecycle managers burn 10–34 %; (4) today's costmap additions (second `/scan_band` source on both costmaps, raytrace 30 m on the 40 m global map) are a likely chunk.
- *FCU now:* `ATC_SPEED_IMAX 0.1`, `ATC_DECEL_MAX 2.0` (undo 1.0 / 0). `max_linear` stays 0.4.

**Q36 — ★ CPU saturation cause found and mostly fixed: 65 mavros plugins (2026-09-26, `083cdf4`)** ✅
- *Root cause:* a **per-process middleware tax** — a do-nothing test node cost **8 % of a core** on domain 0 and **0 %** alone on an empty domain (profile irrelevant: 8.0 vs 7.4 %). It is the cost of tracking the graph, which held **111 nodes — 65 of them mavros** (in ROS 2 each plugin is a node with its own parameter services). Not restart churn: no process had respawned.
- *Fix:* mavros `plugin_denylist ["*"]` + `plugin_allowlist` of the 13 used (sys_status, sys_time, command, param, rc_io, imu, local/global_position, home_position, gps_status, setpoint_velocity, setpoint_raw, waypoint). `mavros_plugins:=all` restores. Add `gps_rtk` for RTCM, `vision_pose` if ExtNav returns.
- *Measured after T1 restart:* **total 495 → 333 %**, mavros **102 → 19 %**, idle-node tax **8.0 → 0.6 %**, collision_monitor_follow 34 → 11 %, graph **111 → 60 nodes**. PREFLIGHT PASS.
- ⚠️ *Measurement pitfalls hit:* `ros2 run` launches the node as a CHILD — timing its PID measured the idle Python launcher (0.0 %); measure the real executable. `/ros_discovery_info` is a raw DDS topic, not reachable from rclpy.
- *Still hot (next):* person_detector 33 %, target_selector 26 %, gps_map_odom 20 %, fcu_odom_relay 13 %, **system_ready 12 %** (a monitor). *To verify on the rover:* follow gate back to 20 Hz, and whether the ~1 s throttle hold after a stop shrinks now mavros isn't saturated.

**Q37 — ★★ Q24/Q34 RESOLVED: the 0.8 m coasting was command latency through a saturated mavros (2026-09-26 15:14, bag `pf_stop_test_151400`)** ✅
Same stop test, same FCU params (IMAX 0.1, DECEL 2.0), only change = mavros 65 → 13 plugins (`083cdf4`). RC flat.

| | mavros saturated (14:44) | **mavros slimmed (15:14)** |
|---|---|---|
| speed at cut | 0.352 m/s | 0.384 m/s |
| **peak roll** | 0.517 m | **0.165 m** |
| throttle +0.5 s | 55 % | **8 %** |
| tracks to neutral | 1.73 s | **0.53 s** |
| follow gate output | 10.8 Hz | **18.4 Hz** |
| zero glitches | 5 | **0** |

- The ~1 s "throttle hold" that no FCU parameter removed was our zero command waiting in a saturated mavros. Stop is back to ~N8's 0.14 m, **well inside the 0.6 m stop zone margin.**
- ⓘ *Unseparated:* `ATC_SPEED_IMAX 0.1` / `ATC_DECEL_MAX 2.0` were tested while mavros was saturated, so their own contribution is unknown. They did not hurt cruise. Keep, or run one control with IMAX 1.0 / DECEL 0 (operator's call).
- ⓘ Lesson: **a symptom that survives every "obvious" parameter change is a hint that the cause is upstream of those parameters.** Three FCU changes produced diminishing returns while the real cause was a CPU queue on the Jetson.
- *Unblocked:* `max_linear` 0.4 → 0.6 can now be tested (re-measure the stop at 0.6).

**Q38 — First 0.6 m/s run: the rover never reached 0.6 — FCU target capped near 0.4 (2026-09-26 15:23, bag `pf_stop_test_152300`)**
- T4 at `max_linear:=0.6` (read back live). 81 % of follow commands were at 0.6; **actual speed while commanded 0.6 (straight): mean 0.372 m/s**, p90 0.44, 0.599 only as a spike.
- **Not throttle-limited:** mean throttle **47 %** (max 53 %) of 100 % available — i.e. the feed-forward for a ~0.4 m/s *target*. The FCU held its own target near 0.4.
- *Likely cap:* **`WP_SPEED = 0.4`** — the Q15 deviation (documented 2.0). Not yet proven that it limits GUIDED velocity commands; the next run decides. `SPEED_MAX 0` (auto ≈ CRUISE_SPEED×100/CRUISE_THROTTLE ≈ 0.89 m/s), `MOT_THR_MAX 100`.
- *Stop in this run* (from 0.366 m/s): **0.133 m**, neutral in 0.49 s, follow gate 20.0 Hz, 0 glitches, RC flat — mavros fix holding.
- *Next:* operator restores `WP_SPEED 2.0` (undo 0.4); repeat the 0.6 run. Also needed for the P6 AUTO benchmark.

**Q39 — ★ 0.6 m/s validated: WP_SPEED was the cap; stop from 0.6 = 0.286 m (2026-09-26 15:35 / 15:44)** ✅
- *Cap confirmed:* operator restored **`WP_SPEED 2.0`** (read back). Same 0.6 run: straight-line speed at a 0.6 command **0.372 → 0.567 m/s**, throttle 47 → 71 %. **So ArduPilot Rover's `WP_SPEED` limits GUIDED velocity commands** — every earlier "0.4 m/s" run was capped by the FCU, not by `max_linear`.
- *"0.916 m/s" was not real:* one EK3 velocity sample; throttle steady at 70 %, neighbours ~0.6, fused odom 0.599 at that instant. GPS-velocity noise in the walled lane. **No real overshoot.** (Use an odometry-position derivative over ~1 s for speed, not single EK3 samples.)
- *Stop from 0.6 (`DURATION_S=7`, bag `pf_stop_test_154400`):* setpoint 0.600, actual 0.60–0.63 m/s at the cut → **peak roll 0.286 m, tracks neutral 0.71 s**; RC flat, gate 20.1 Hz, 0 glitches. **~0.31 m inside the 0.6 m stop-zone margin.** (0.165 m from 0.38 m/s for comparison.)
- *Protocol lesson:* the end-of-run cut lands after the operator slows at the lane's end unless `DURATION_S` puts it in the first steady stretch (here 5–8 s after enable).
- *FCU now:* `WP_SPEED 2.0` (restored, also fixes P6 AUTO), `ATC_SPEED_IMAX 0.1`, `ATC_DECEL_MAX 2.0`, CRUISE 0.4/45. T4 run at `max_linear:=0.6`; default still 0.4 pending the operator's decision.

**Q40 — SLAM chosen from measurements: 2D slam_toolbox, running in shadow mode (2026-09-26)** ✅ *step 1 of 4*
- **No usable IMU for LiDAR-inertial SLAM:** RS-Helios-16P has none; the Cube **denies** RAW_IMU above 10 Hz over MAVLink (`SET_MESSAGE_INTERVAL` at 50 Hz → `MAV_RESULT 2`; `SCHED_LOOP_RATE 50`, `SERIAL0_BAUD 115`). Raising the FCU loop rate for this would be a large, risky change → FAST-LIO2 / LIO-SAM need an added IMU (≥200 Hz, rigid on the LiDAR bracket).
- *Choice:* **slam_toolbox 2D** now (installed, no IMU, 3.7 % CPU, Nav2-native, indoor + outdoor on reasonably flat ground; fed by the ground-segmented `/scan`). *3D later:* LiDAR-only with our fused odometry as a motion prior (e.g. LeGO-LOAM-style, KISS-ICP/GLIM with odom prior — risks: pivots on 16 beams, the corridor-like lane), or FAST-LIO2 + IMU. Visual (ORB-SLAM3/VINS stereo on the D435 IR pair) possible but CPU-costly. Rejected: mrg_slam (too heavy, C2), LIO-SAM (IMU), RTAB-Map/depth (sunlight). Complement: **RTK corrections** over the already-wired RFD868 for cm-level outdoor position.
- *Shadow mode (`slam.launch.py`, default):* map frame `slam_map`, `transform_publish_period 0`, outputs `/slam/*`. **Verified by counting messages:** only `gps_map_odom`'s `map->odom` (301/15 s) reached `/tf`; slam_toolbox registers a TF publisher but sends nothing. Built a 30×22 m map stationary.
- ⓘ `install/dagger_slam` is a pre-reorg leftover (source removed; a previous developer's indoor "factory" maps/configs) — not ours, not used.
- *Next:* (2) operator drives a loop on the RC with shadow SLAM + `tools/slam_record.sh`, save map, inspect; (3) `map_source` switch so SLAM owns `map→odom` in localization mode with gps_map_odom off; (4) GNSS↔map alignment for GPS goals.

**Q41 — First mapping drives: lane1, lane2; viewer TF; lane2 drive NOT recorded (2026-09-26)** 🟡 *step 2 of 4*
- *Foxglove couldn't show the shadow map:* `slam_map` is not connected to the TF tree by design (shadow mode). **Fix (eb5a1ed):** `slam_pose_tf` republishes `/slam/pose` as `slam_map→slam_base_footprint` (5 Hz), a separate branch — nothing new publishes into `odom` or `map` (counted over 8 s: `slam_map→slam_base_footprint` 52, `map→odom` 161, nothing else into `odom`). Foxglove display frame `slam_map`.
- *`save_map` service failed* because the map topic is remapped → save with `ros2 run nav2_map_server map_saver_cli -t /slam/map -f <path> --ros-args -p map_subscribe_transient_local:=true`; pose graph via `/slam_toolbox/serialize_map`. Saved in `src/dagger_bringup/maps/` as `lane1.*`, `lane2.*` (+ `*_view.png`).
- *lane1:* user saw the "rover body" in the map. 0 hull returns in all 5,202 scans (`min_laser_range 0.5`) → most likely the user standing next to the rover; the rover sat still 6.7 of 8.7 min, so a standing person gets integrated.
- *lane2:* 829×897 px @ 0.05 m, 7,105 occupied cells. User: "decently good, very slight error near turns". Walls show short step/notch segments — either real building features or turn misregistration; **not separable without the drive's data.**
- 🔴 **The lane2 drive was not recorded.** `slam_record.sh lane2 clouds` was refused (`Output folder … slam_lane2 already exists`); the existing `slam_lane2` bag is an earlier 81 s start (7.0 m, 39° of turning), not the mapping drive. Same trap as follow_solo in Q16, not carried over. **Fix (65dc5a0):** bag names get `_HHMMSS`.
- *Turn-error candidates to test offline on a re-recorded drive:* (a) pivots — a 16-beam scan without deskew smears during in-place turns (10 Hz scan at ~0.5 rad/s = ~3° within one sweep); (b) `minimum_travel_heading 0.3` (17°) means few scans are inserted mid-turn, so matching across a pivot leans on the odom prior; (c) odom yaw during pivots (track slip). Offline levers: lower `minimum_travel_heading` to ~0.1, raise `angle_variance_penalty`/correlation search, slow pivots on the RC.

**Q42 — Turn error root cause: slam_toolbox takes NO scans during pivots (lane3, 2026-09-26 16:48)** 🟡
- *Drive:* `slam_lane3_164840` (119 s, clouds), 41.2 m path, 462° total turning, ended 1.84 m from start; map saved as `maps/lane3.*`.
- *Measured (`/slam/pose` vs `/odometry/filtered`):* both pivots produced **no SLAM update for ~10 s** (48.3→58.1 s: 146° turned, 0.27 m net move; 97.7→106.2 s: 154°, 0.32 m). Scans were fine throughout (≥497 valid points, median 586). The first pose after each pivot came out **~19–20° off odometry**. On the straights the SLAM–odom yaw difference stays within 0–2°. After the first pivot SLAM snapped back by +15° at 68.5 s; the scans added before that were inserted rotated → the wall notches near turns.
- *Why:* scans are accepted on **distance travelled** (net 0.27 m ≈ the 0.3 m threshold, ×0.8); `minimum_travel_heading` does not trigger processing in this build. After a pivot, the first scan is matched across ~150° with only the odom prior; the ±20° coarse search (`coarse_search_angle_offset 0.349`) lets it lock onto a wrong local minimum ~19° off.
- **Offline replay (`tools/slam_replay.sh`, own ROS domain 77, sim time, 1x; scored by `tools/slam_replay_eval.py`)** — the baseline reproduces the live run exactly (91 vs 90 poses, same gaps):

  | run | poses | longest pivot gap | worst yaw jump | SLAM−odom yaw | CPU |
  |---|---|---|---|---|---|
  | base (0.3 m, 0.5 s) | 91 | 9.8 s / 146° | 19.4° | −21.6…0° | 17 % |
  | dist 0.1 | 153 | 6.1 s / 127° | 17.7° | −18.7…0° | 24 % |
  | dist 0.05 | 160 | 5.4 s / 118° | 7.0° | −7.8…0° | 25 % |
  | dist 0, 0.5 s | 213 | 0.6 s | 1.3° | −3.8…0° | 33 % |
  | **dist 0, 1.0 s** ✅ | 111 | 1.1 s | 1.8° | −4.1…0° | 19 % |

  A true pivot moves the rover only ~0.05 m, so **no distance threshold fixes it**; turning the distance gate off and pacing by time does. Map: single clean walls, no doubled wall, 16 % fewer occupied cells (`maps/lane3_replay_compare.png`).
- **Fix:** `minimum_travel_distance 0.0`, `minimum_time_interval 1.0` in `slam_toolbox.yaml`. ⚠ Trade-off: scans are also added while parked (1/s), so people walking past a parked rover can get into the map (the lane1 effect). If that matters: pause mapping while parked (`/slam_toolbox/pause_new_measurements`), later.

**Q43 — SLAM step 3: localization on the saved map owns map->odom; GNSS + scan search seeds it (2026-09-26)** 🟡 *offline-validated, not yet live*
- *Map:* `lane3fix` = the lane3 drive re-mapped with the pivot fix (Q42), georeferenced by `tools/slam_georef.py` → `lane3fix_geo.yaml` (datum lat/lon + ENU; rotation from the dual-antenna heading +1.65°; GNSS-vs-SLAM residual 1.09 m rms on that drive). The ArduPilot local frame cannot be the anchor: its origin re-sets on every FCU boot.
- *Switches:* T5 `slam.launch.py mode:=localization map:=lane3fix` (localization node, `map_frame map`, publishes map->odom); T2 `nav2_gps.launch.py map_source:=slam` (no gps_map_odom). Default stays `map_source:=gps` / shadow mapping. Preflight now checks **exactly one** map->odom owner and, when SLAM owns it, `/slam/localized`.
- 🔴 *First attempt, GNSS-only seed, failed:* on lane2 slam_toolbox locked **20° / 4.5 m wrong** (20° = the edge of its ±0.349 rad coarse search). A brute-force scan-to-map fit showed the truth: the dual-antenna HEADING was right (38°), but GNSS POSITION was **~5 m off** the map in that session (lane1: 2.4 m). No RTK + buildings → between-session GNSS bias of metres, beyond what slam_toolbox localization recovers from.
- **Fix — `slam_geo_init` + `scan_map_match.py`:** GNSS (10 still fixes, averaged) only centres a search: ±8 m, ±10° around the GNSS heading, on the saved map's distance transform; accepted only if score ≤ 0.15 m and the best pose >1.5 m away is ≥2× worse (corridor aliasing); winner → `/initialpose`. Retries on a newer scan; Foxglove's pose estimate overrides. Health: scan-to-map score at SLAM's pose every 2 s on `/slam/match_score`; `/slam/localized` true after 3 checks ≤ 0.20 m, false after 3 ≥ 0.35 m. Cost: search 1.1 s once, health check 0.09 ms.
- **Offline results** (`tools/slam_loc_replay.sh`, domain 77, bag map->odom filtered out by `tools/tf_filter.py`):

  | recording | seed moved from GNSS | stays locked? (match score median / max) | notes |
  |---|---|---|---|
  | lane3 (map's own drive) | 0.6 m | 0.054 / 0.144 m | pose within 5 cm / 0.2° of the mapping truth at the seed |
  | lane3 + 3 m GNSS offset | 2.5 m | 0.052 / 0.143 m | same seed pose |
  | lane3 + 6 m GNSS offset | 5.8 m | same | same seed pose |
  | lane2 (81 s) | 4.7 m | 0.089 / 0.108 m | = brute-force truth; GNSS bias 4.3 m median |
  | lane1 (9 min) | 2.5 m | 0.034 / 0.350 m (1 of 252 checks) | never LOST |
- *Consequence for step 4 (GPS goals on the SLAM map):* lat/lon → map carries the session's GNSS bias (2–5 m here). Fine for "go to the far end of the lane", not for precise points — RTK over the RFD868 would remove it.
- ✅ **Live, parked (17:49):** seeded 1.4 m from GNSS (score 0.054, next-best 0.215), localized in 6 s, preflight PASS with owner `slam_toolbox (localization)`, live score 0.065 m.
- *CPU, parked:* slam_toolbox localization **34 %** (matches a scan every 1 s even parked — cost of the Q42 pivot fix; measure while driving before tuning); `slam_geo_init` **25 %** → its Python TF listener decoded all of /tf for a 2 s check. **Fixed:** health scores the scan SLAM processed at the pose `/slam/pose` gives it (exact stamp) — no TF listener; offline scores also tighter (median 0.030 m vs 0.052). `map_start_at_dock` removed (the localization node warns it is unsupported). `slam_record.sh` now also records `/slam/match_score`, `/slam/localized`, `/slam/gnss_pose`, `/initialpose`, `/mavros/rc/in`.
- *Next:* RC drive in localization mode with `slam_record.sh`; then a Nav2 goal sent by the user.

**Q44 — First live drive in SLAM localization (loc1, 2026-09-26 18:01)** 🔴 *INVALID as a localization test — see Q45*
> ⚠ **Correction (same day):** this drive ran on build 0b70017, which removed `map_start_at_dock` and so made the localization node **skip loading the map** (`Map starting pose not specified`). slam_toolbox only tracked scan-to-scan from the (correct) seed. The numbers below describe that, not map localization — the along-lane sliding is what an unanchored tracker does. The recording itself (scans, odom, GNSS) is still valid input for replays.
- *Drive (RC):* `slam_loc1_180107`, 165 s, 41.6 m, 514° of turning incl. pivots, up to 0.84 m/s, and past the end of the mapped area.
- ✅ **Stayed localized throughout:** match score median 0.103 m, p90 0.150, max 0.282 (at the far end, beyond the map) — never ≥ 0.35 → never LOST.
- *SLAM vs GNSS:* 4.6 m median, mostly ACROSS the lane (3.8 m). SLAM cannot be 3.8 m off sideways while scans fit the walls to 0.1 m → it is this session's GNSS bias (same as lane1 2.4 m, lane2 4.3 m).
- ⚠ **map->odom jitter while moving:** per SLAM update, parked median 0.00 m; driving median 0.10, p90 0.30, max 0.58 m; pivoting median 0.24 m. Two sources: past the map end (+70–77 s, 0.46–0.58 m, expected), and **mid-lane sliding along the lane axis** (+86–106 s: 0.3 m corrections ≈ the odom travel, no yaw change) — a plain corridor constrains the along-lane position weakly, and SLAM takes it from the scan rather than odom. Matters because goals live in `map` and the goal tolerance is 0.4 m. Offline tuning on this drive → next entry.
- *CPU, parked:* slam_toolbox localization 23–34 %; `slam_geo_init` 25 % → **13.5 %** after the TF-listener removal (0b70017). CPU while driving not captured (my logger matched its own shell — H-pitfall below).

**Q45 — My regression: localization silently ran WITHOUT the map (0b70017 → fixed) (2026-09-26)** ✅
- *Cause:* to silence the "dock is not supported" warning I removed `map_start_at_dock`; with neither `map_start_at_dock` nor `map_start_pose`, the localization node logs one ERROR and does **not** load the pose graph, then happily tracks from `/initialpose`. The scan-to-map health score did not catch it: a correct seed + locally consistent scan-to-scan tracking fits the saved map image for minutes (loc1: max 0.28 m). Also invalidated: `l3_v2` and the loc1 tuning runs (`t_*`).
- **Fix:** `map_start_pose: [0, 0, 0]` (launch + replay; the real pose still comes from `/initialpose`) → "Load From File" again. **Guard:** `slam_geo_init` compares slam_toolbox's `/slam/map` with the saved map and refuses `/slam/localized` unless ≥ 60 % of the saved occupied cells are there. Verified both ways offline: loaded → 100 % → localized; map unloadable → slam_toolbox builds its own (23–26 %) → "NOT localized" error every 30 s. Preflight inherits it via `/slam/localized`.

**Q46 — loc1 replayed WITH the map: along-lane correction is real; settings kept (2026-09-29)** 🟡
- Both runs `map LOADED 100 %`, seeded 4.7 m from GNSS (-3.58, -4.82, 40.2°), localized, never LOST.

  | run | driving corrections > 0.2 m | worst | match score median / max |
  |---|---|---|---|
  | `m_base` (current settings) | 30 / 97 | 0.56 m | **0.090** / 0.747 m (1 of 76 checks ≥ 0.35) |
  | `m_odom` (`distance_variance_penalty 0.2`, `minimum_distance_penalty 0.2`) | **18 / 97** | 1.11 m | 0.110 / 0.326 m |
- With the map anchoring it, SLAM still moves map->odom by 0.1 m (median) per 1 s update while driving and 0.2 m on pivots. Trusting odom more halves the count but fits the map worse and has a larger worst jump → **kept current settings**. Part of it is likely genuine odom error being corrected (track slip on pivots).
- *If the first Nav2 goal dithers at the 0.4 m tolerance:* smooth SLAM's map->odom like gps_map_odom smooths GNSS (slam_toolbox `transform_publish_period 0`, a node composes /slam/pose with odom at its stamp, gates pivots, low-passes).
- *Bench:* `slam_loc_replay.sh` takes `geo.<param>:=` for the seeder — loc1 starts moving ~9 s in, so the 10-still-fix seed only just fits (`m_odom` needed `geo.max_speed:=1.0`).

**Q47 — ✅ loc2: first VALID live drive in SLAM localization (2026-09-29 11:04)**
- Parked checks first: `/slam/localized` true, `/slam/map` 7,009 occupied cells vs saved 6,934 (map loaded), preflight PASS (user). (A stale ros2 daemon showed `/slam/match_score` "not published" — H12; it runs at 0.5 Hz.)
- *Drive (RC):* `slam_loc2_110448`, 146 s, 43.0 m, 451° turning, ≤0.67 m/s, past both map ends. `/slam/localized` true throughout (one message, never flipped).
- **Match score** median **0.050**, p90 0.110, max 0.142 m — never ≥ 0.20 (loc1 without the map: 0.103 / 0.282).
- **Rover map-pose jump vs odom prediction, per SLAM update** (the metric that matters to Nav2; the map->odom metric of Q44/Q46 inflates yaw corrections by the distance from the odom origin): driving median **0.04 m**, p90 0.18, max 0.30 m, yaw median 0.2°; pivots median 0.21 m (likely real lateral track skid odom does not see), yaw 0.6°; parked 0.00 m. All inside the 0.4 m goal tolerance → **no map->odom smoothing needed.**
- SLAM vs GNSS today ~1.4 m (along −1.2, across −0.8).
- *CPU while driving* (`tools/node_cpu.py`, by PID, H19-safe): slam_toolbox localization median **28 %**, max 42 %; slam_geo_init 21 %.
- `pf_record.sh` now timestamps bag names and records the SLAM topics (3f44628).
- *Next:* first Nav2 goal in the SLAM map frame: `send_goal.py --to "0.59,-1.37,39.5" --frame map` (6 m down the lane, 0.8 m left of the start line: ≥1.44 m from mapped walls along the straight line, live costmap right side tight at 0.7 m).

**Q48 — ✅ First Nav2 goal in the SLAM map frame: succeeded, 0.23 m (2026-09-29 11:17)**
- `pf_slam_goal1_111635`; goal `send_goal.py --to "0.59,-1.37,39.5" --frame map` (6 m down the lane, 0.8 m left). GUIDED + armed by the user; **no RC stick input** (ch1/ch2 never > 60 µs off 1500).
- bt_navigator "Goal succeeded" after **16.8 s**; 5.58 m driven, cmd ≤ 0.40 m/s (odom ≤ 0.51), |wz| ≤ 0.16.
- **Final pose on the map:** (0.41, −1.51, 49.5°) → **0.23 m short** along the goal heading, 0.01 m lateral, yaw +10.0° (tolerance 0.4 m / 0.2 rad = 11.5°).
- **Goal stability:** 17 replans in `odom`; the goal's end point moved ≤ 0.22 m over the run (costmap cells are 0.1 m, so part is grid quantisation) — no dithering near the end. Match score during the run median 0.031, max 0.055 m.
- ⓘ The +10° yaw at the finish is the controller, not SLAM (SLAM yaw jitter while driving is ~0.2°); it is inside the goal checker's tolerance.

**Q49 — SLAM step 4: lat/lon goals in the SLAM map (2026-09-29)** ✅ *live-verified 11:26*
- `tools/gps_goal.py` detects who owns map->odom (`gps_map_odom` → GPS mode as before; `slam_geo_init` → SLAM mode) and in SLAM mode converts through the running map's `_geo.yaml` (read from slam_geo_init's `geo_file`). Anchors: `--anchor geo` (default; fixed: same lat/lon → same map point; error = the georeference, ~1 m + mapping-day GNSS bias) and `--anchor live` (rover SLAM pose + offset from the current fix; error = today's GNSS bias). `--enu` is relative to the SLAM pose (no GNSS position error).
- `dagger_bringup/geo_map.py` (`GeoMap`: to_map, to_latlon, enu_to_map_vec) is now the single lat/lon↔map conversion for gps_goal and slam_geo_init. slam_geo_init re-checked on the lane3 replay after the move: identical seed pose, map loaded 100 %, localized.
- *Dry runs (live stack, SLAM mode):* datum lat/lon → map (1.73, −0.33) = R⁻¹(−t) ✓; live anchor puts the same point 1.5 m away (today's GNSS bias vs the mapping day); `--enu 3.54,3.54` → 5.0 m at 45° ✓; map→lat/lon→map round trip exact.
- *Live test prepared:* goal 1's endpoint as lat/lon `26.74792563,75.85532663` (known clear) → `gps_goal.py --ll …` from the rover at (−2.47, −4.65).

**Q50 — ✅ First lat/lon goal in the SLAM map: succeeded (2026-09-29 11:26)**
- `pf_slam_ll1_112626`; `gps_goal.py --ll "26.74792563,75.85532663"` (anchor geo) → map (0.59, −1.37), 4.5 m. No RC stick input. Goal succeeded after 13.7 s, cmd ≤ 0.40 m/s. `RESULT SUCCEEDED | map error 0.38 m | GPS distance 1.54 m` (the GPS number is today's GNSS bias, not a miss).
- Stopped at (0.45, −1.73, 56°): **0.36 m short along the approach, 0.14 m right**; 0.09 m of coasting after the last command. Match score median 0.035, max 0.058 m.
- *Why short (both goals, 0.23 / 0.36 m):* the goal checker succeeds on entering the **0.4 m xy tolerance** circle, approached from behind; where inside it the rover stops also moves with the goal's re-projection into odom (plan end moved ≤ 0.36 m over the run — SLAM map->odom corrections + 0.1 m costmap cells). Precision is therefore set by `xy_goal_tolerance`, not by SLAM. Tightening it (e.g. 0.25) is a user decision — not changed.
- **SLAM plan steps 1–4 done** (shadow mapping, pivot-safe mapping, localization owning map->odom with GNSS+scan seeding and health, Nav2 map-frame and lat/lon goals).

**Q51 — ✅ Round trip with turn-around in the SLAM map, lat/lon waypoints (2026-09-29 11:33)**
- `pf_slam_rt1_113332`; `gps_goal.py --ll "<wp1> <wp2>"` → NavigateThroughPoses: wp1 (7.86, 5.91) near the far end (3.1 m wall clearance, chosen from the mapping drive's return leg), wp2 = the start point (−2.46, −4.63). No RC input.
- **Succeeded in 61 s:** 20.4 m, 222° of turning incl. a ~180° pivot (+65.7…+72.5 s, |wz| ≤ 0.45). Turned for home 1.56 m before wp1 — by design (`RemovePassedGoals radius 2.0`, 2026-09-16).
- **Final:** 0.44 m at "Goal succeeded" (rover still at 0.12 m/s; the checker fired at 0.4), settled **0.36 m short along the approach, 0.02 m lateral** — same goal-tolerance pattern as Q48/Q50.
- **SLAM through the pivot:** rover map-pose jump vs odom — driving median 0.05 m, max 0.17 m; **pivot median 0.045 m, max 0.09 m**, yaw ≤ 2.8°. Match score median 0.034, max 0.082 m. Localized throughout.
- **GNSS today was bad and SLAM did not care:** SLAM vs GNSS median **5.5 m, max 12.0 m** during the run (the start fix was ~7 m from the morning's at the same spot). In GPS mode (gps_map_odom) this run's goals would have drifted with it.
- ⚠ Recorder kept running after the user's Ctrl-C attempt; bag unreadable until stopped (H14) — no data lost.

**Q52 — Click-to-goal in Foxglove made safe: goal_pose_relay (2026-09-29)** ✅ *live-verified 12:02*
- *Problems found before use:* (1) Foxglove stamps a clicked goal with the click time; Nav2 re-transforms the goal (map) into the costmap frame (odom) at that stamp every replan → TF buffer (~10 s) ages out → abort — the 2026-09-14 failure that made send_goal/gps_goal send stamp 0. (2) `/initialpose` was not client-publishable, so the "Foxglove pose estimate overrides the SLAM seed" claim (Q43) was not true.
- **Fix (be9f5e8):** Foxglove publishes `/goal_pose_ui` and `/initialpose_ui` only (plus `/estop`); `goal_pose_relay` (started by foxglove.launch.py) forwards goals with stamp 0, converts clicks in moving frames into map, refuses goals while `/slam/localized` is false; converts pose estimates into map for slam_toolbox. `/goal_pose` no longer client-publishable.
- *Isolated test (domain 77, static TF, no Nav2):* map click → map, stamp 0 ✓; base_footprint click 1 m ahead → correct map point (4, 3, 90°) ✓; odom click → odom, stamp 0 ✓; not localized → refused ✓; pose estimate in odom → map (1, 2) ✓.
- Cancel a running goal from the CLI: `ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"` (zero id = cancel all).
- *Dry test (HOLD) found:* Foxglove would not draw `/plan` — "Path poses must all have the same frame_id. odom != ''": Humble's **Theta*** leaves each pose's frame_id empty (header says odom). Controller uses the header → driving unaffected. **Fix:** the relay republishes `/plan_viz` with per-pose frames — view that. (Built by the user while my tool calls were blocked by a failing command classifier.)
- **Live (`pf_slam_click1_120245`), three clicked goals in the map frame, no RC input, all SUCCEEDED:** 9.3 m in 32.7 s → 0.35 m (yaw +5.4°); 8.4 m incl. an initial pivot in 31.1 s → 0.30 m (−3.3°); 1.8 m in 7.6 s → 0.31 m (−2.0°). None aborted past the old ~10 s TF-age limit. Match score median 0.066, max 0.115 m. Stops 0.2–0.3 m short along the approach = the 0.4 m goal-tolerance pattern (Q48/Q50/Q51).
- *Foxglove setup:* 3D panel display frame `map`; Publish → Pose topic `/goal_pose_ui`, Pose estimate topic `/initialpose_ui`; show `/plan_viz` (not `/plan`).

**Q53 — CPU pass on the Python nodes (2026-09-29)** ✅ *live-verified back to back after the T1 restart*
- *Live baseline (SLAM-mode stack + Foxglove, T4 off):* system 327 % of 600 %. foxglove_bridge 48.5 % (the viewer), slam_toolbox 27.3 %, mavros 17.1 %, **goal_pose_relay 16.5 %**, patchworkpp 14.7 %, **slam_geo_init 13.0 %**, 3× pointcloud_to_laserscan 32.5 % total, **fcu_odom_relay 11.9 %**, **system_ready 11.7 %**, Nav2 servers 7.5–9.5 % each, **scan_watchdog ×2 14.6 %**, **cmd_vel_safety 7.7 %**, **joint_state_publisher 5.7 %**, gps_heading_imu 4.7 %.
- **Round 1 (3a0f55d):** system_ready — raw subscription to the 0.8 MB × 10 Hz cloud (only its arrival time is used); goal_pose_relay — TF listener removed (goals outside map/odom and pose estimates outside map refused); slam_geo_init — numpy map count, odometry dropped once seeded; scan_watchdog — raw /scan; joint_state_publisher 10 → 2 Hz (only wheel joints move). Relay re-tested isolated; slam_geo_init re-checked on the lane3 replay.
- **target_selector (follow-me):** Python TF listener replaced by /tf_static + /odometry/filtered (0.5 s staleness check). Follow tests unchanged: 4/4, 3/3, 12/12.
- **person_detector — no change kept:** raw subscribe + throttle-before-decode measured 29.6 → 28.7 % (noise; rclpy decodes images in C) → reverted. Where its time goes (per frame): rotate 0.8 ms, preprocess 3.1 ms wall / 4.6 ms CPU, **TensorRT infer 9.7 ms wall / 5.0 ms CPU** (CPU busy half the GPU time — CUDA sync), decode 1.4 ms. Possible next: blocking-sync CUDA flag, GPU-side preprocessing — not done.
- *Pitfall repeated:* timing a `ros2 run` PID measures the launcher (0 %) — measure the installed executable.
- **Correction + round 2 (772e6ce), live T3 restart 13:10:** relay confirmed < 1 % live (was 16.5 %). But the **raw-subscription changes in round 1 were ineffective**: A/B in an isolated domain, system_ready with a 0.8 MB × 10 Hz cloud 9.8 % (decoded) vs 9.4 % (raw) — rclpy decodes in C; the cost is *receiving* the cloud (without the subscription: 0.6 %). system_ready now checks the LiDAR driver via the ROS graph (`count_publishers`) → **13.2 % → 0.4 %**; /system_ready only feeds the T1 console (preflight still checks the cloud ≥ 8 Hz). scan_watchdog's raw /scan is harmless but gains nothing. The two scan watchdogs pay the same cloud-receive cost (12.8 % / 7.9 %) — left, because their cloud check prevents pointless p2l restarts.
- **joint_state_publisher was never slowed:** the 2 Hz edit went into `description.launch.py`, but T1 (`drive_bringup`) includes `dagger_description/launch/rsp.launch.py`, which set **20 Hz** (9.5 % live). Fixed there (772e6ce).
- **Live, back to back (same session, T1 restarted + preflight):** system 353 → **335 %**; system_ready 19.7 → **7.5 %**; joint_state_publisher 9.5 → **1.3 %**; goal_pose_relay 16.5 → < 1 %. target_selector to be measured with T4.
- ⚠ **Comparing CPU % across sessions is unreliable here:** governor `schedutil`, cores averaged ~830 MHz of 1510 (15 W mode, 55 °C, no throttling) → the same work reads as a higher %. Unchanged nodes read ~1.5× this morning's values (mavros 17 → 29 %). Compare only A/B in the same conditions (isolated bench, or back-to-back live).

**Q54 — Click-to-select person following, phase 1: browser video + person IDs (2026-09-29)** 🟡 *built + offline-tested (2189f31); live test pending*
- *Request:* follow a person chosen by clicking them in low-latency video on the PC, in crowds; low latency, reliable. *Design (approved):* browser page served by the rover (T4, `http://<rover>:8080`), per-frame JPEG over WebSocket (the **Orin Nano has no NVENC**; software x264 = +54 % CPU; JPEG encode 3–5 ms), latest-frame-only, image + its boxes in one message, click sends the track ID; all tracking/control onboard; deadman stop on link loss. Phases: P1 UI + IDs → P2 LiDAR/odom multi-person tracker drives /follow/target from the selection → P3 OSNet re-ID (TensorRT) → P4 real-crowd bag tuning → P5 live.
- **Pitfalls found (all fixed):** (1) Ubuntu 22.04 `python3-websockets` 9.1 passes `asyncio.Lock(loop=None)` → **cannot complete a connection on Python 3.10**; it also truncates HTTP bodies from its handshake hook → switched to **`python3-tornado` 6.1** (verified from the Jammy .deb before asking for the install). (2) tornado keepalive ping 5 s / timeout 5 s closed every page after exactly 5 s → 2 s / 6 s. (3) Latest-only was defeated by the kernel send buffer (19 stale frames after a 2 s stall) → per-page SO_SNDBUF cap (tornado keeps the socket on `ws_connection.stream` at open()) → 2 stale frames, fresh after 7 ms. (4) **rclpy: assigning `bytes` to a uint8[] field costs 32 ms per 166 KB message** (per-byte validation) vs 0.11 ms with `array('B')` — the detector cost +31 % with a viewer before, +8 % after. (5) rclpy's own Ctrl-C handler killed the context before ui_server could publish "follow off" → own signal handling. (6) destroying a node under a running executor thread aborted the process → shut the executor down first.
- *Tests:* `tools/test_person_tracking.py` 12/12 (IDs never reused, selection never moves, LOST, weak detections); `tools/test_ui_server.py` 10/10 (isolated domain 77, fake camera + real nodes: pairing, select, FOLLOW, deadman 1.43 s / disconnect 0.2 s, stall recovery). ui_server adds ~2–4 ms; person_tracker 4.2 ms/frame.
- *Needs:* `sudo apt install python3-tornado` (user). `python3-websockets` was installed by the user before the incompatibility was found — unused, removable.

**Q55 — Click-to-follow phase 2: the clicked person drives /follow/target (2026-09-29)** 🟡 *built + scene-tested (ab032df); live test pending*
- `person_tracker` tracks every person in odom (camera bearing + LiDAR range — target_selector's geometry/params, now shared in `person_tracking.py`) and publishes `/follow/target` + `/follow/track_id` (first) + `/follow/target_source` for the SELECTED person only. Association: position decides where both sides are ranged (gated), IoU otherwise. LiDAR hold for the selected person out of view (5 s), refusing wall-sized clusters.
- 🔴→✅ **Found by the unit test:** when the held target vanished (walked behind someone), the growing gate (up to 1.6 m) slid the hold onto the bystander 1.5 m away. target_selector has the same weakness but only knows one person; the tracker now refuses any cluster within 0.6 m of another tracked person.
- 🔴→✅ **Found by the scene test:** rclpy raises *"Logger severity cannot be changed between calls"* when one source line logs at two levels → person_tracker crashed at its first state change (tracking stopped; failed safe: no target). One call site per level; same pattern removed from slam_geo_init. person_tracker / ui_server now respawn.
- Never auto-selects or hands over; LOST carries the reason. Selecting someone else while following changes the track id → follow_controller disables (by design). `follow_controller` publishes `/follow/state` (+ last disable reason), publishing only.
- `follow.launch target_source:=tracker` (default, click) | `selector` (old automatic, for follow_solo.sh) — one /follow/target publisher either way.
- *Tests:* test_person_tracking 26/26; **test_click_follow** (real person_tracker + follow_controller, simulated LiDAR/camera with occlusion, scripted click + FOLLOW): main (out of view L/R, bystander, expiry → LOST, controller disables, never drives at B) / cross (3 m vs 6 m crossing, ids kept) / occlude (A behind B → LOST, B never the target) / wall (refused, never drives) — 21/21 on three consecutive runs (one earlier flake = the scene ending at the same instant as the controller's 6 s search timeout; scene lengthened). Old path unchanged: test_follow_lidar 4/4 + 3/3, controller 12/12, test_ui_server 10/10.
- *Known limit → phase 3:* a person fully hidden behind another for > ~1 s is LOST (safe), and on reappearing gets a new id; appearance re-ID is what can re-link them.

**Q56 — Click-to-follow: ID stability, re-identification, search, true positives only (2026-09-29)** 🟡 *offline-proven on a real recording; live test pending* (e429080, abed3b1, b0b0ef2)
- *Live finding (user):* IDs changed for the same person. Log: 25 ids in ~100 s for 1–3 people; the selected person flapped TRACKING/HOLD while alone in view. **Cause:** single-frame LiDAR range glitches (range_at on legs / background / a pole) failed the position gate and started a NEW track at once; side/back views dropped below the 0.45 detector threshold → track deleted after 1 s.
- *Tools:* `tools/track_record.sh <name>` (tracker inputs + UI JPEGs), `tools/replay_tracker.py <bag> [--set k=v] [--reid --select-at T]` (re-runs person_tracking offline, counts ids / switches / re-identifications).
- *Recording `track_ids1_163706` (175 s):* old tracker 17 ids / 9 switches → ID-stability fixes 6 / 0 (det 0.45), 8 / 1 (det 0.2) → + re-ID **4 ids, the operator = ONE id through the whole session** (re-identified 21 s after walking off, appearance 0.73; held on LiDAR 16 s beside the rover). Contact sheet showed the operator split into #1/#3/#6/#8 before re-ID, and the false positive = an orange sign (weak 0.2–0.3, never ranged).
- *ID stability:* disputed ranges (strong box overlap keeps the id, range ignored, re-anchor after 3 frames, never onto another person); one LiDAR cluster = one person (nearer box keeps it); ByteTrack two-stage (weak dets only extend confirmed tracks); tentative until 3 confident hits; duplicate suppression; 2 s survival.
- *Re-ID:* OSNet x0.25 MSMT17 ONNX (HF anriha, sha256-pinned) → TensorRT FP16, 8.2 ms / batch 16, ~3.3 ms per person incl. preprocessing; crops from the UPRIGHT frame. Real data: same person across gaps 0.75 median (p10 0.65) vs others 0.51 (p90 0.57, max 0.63). Re-link rule: LOST target, within 30 s, only people who appeared after the loss, ≥ 0.66 for 3 frames, ≥ 0.08 margin → old id back.
- *Operator policy:* **resume following automatically if re-identified within 30 s**; after that disable. *Search:* tracker `/follow/search_hint`; controller `resume_window` (30 s in click mode, 0 = old behaviour) faces the hint, waits 3 s, sweeps one way, rotation only.
- *True positives / never lose:* confident detections only confirm; LiDAR range must be person-sized and not a slice of a wall; only LiDAR-ranged people selectable; isolated LiDAR hold up to 30 s (beside the rover — the live loss at 121 s); 'near' target when closer than the LiDAR's 1 m minimum (the live loss at 131 s): hold + face, never drive.
- *Found by tests and fixed:* hold sliding onto a bystander when the target vanished; A behind B re-anchored onto B; one-line two-severity logging crash (rclpy); a sign in front of a wall taking the wall's range; my own test-scene / check bugs (teleporting person, weaker look-alike, camera-bearing offset, scene ending at the controller timeout).
- *Tests:* unit 57/57; test_click_follow 8 scenes 45/45 twice; old path test_follow_lidar 4/4 + 3/3; controller 12/12; test_ui_server 10/10.

**Q57 — reid1 live test: slow search turns, lost beside the rover, a plant as a person (2026-09-30)** ✅ *fixed d5c243a; search speed live-confirmed in reid2 (Q58)*
- *User:* "when searching it is rotating very slow; if the target gets too close to the rover it lost the target; it detects a plant as a person". Recording `track_reid1_105438`.
- 🔴→✅ **Search turned at 0.16 rad/s** (commanded 0.45): Nav2 Humble's collision-monitor **slowdown scales the WHOLE twist**, and a plant or wall 0.85–2.6 m ahead occupied PolygonSlow in 71–90 % of both searches (0.41–0.46 rad/s when clear). PolygonSlow removed from `collision_monitor_follow.yaml` (STOP zone + footprint approach kept — the approach simulates rotation). The same zone ×0.4 is now applied to **forward speed only**, in follow_controller (`slow_zone [0.85, 2.60, 0.85]`, instant down / ramped up, stale scan = occupied).
- 🔴→✅ **Lost standing 1.7–2.6 m beside the rover:** the long LiDAR hold required nothing within 1 m, and a small object / plant-bed edge 0.6–0.9 m away ended it. Now within 3 m of the rover (or isolated) the hold runs up to 30 s while the held cluster moves **continuously** (a jump > 0.4 m ends it — it cannot slide onto clutter when the person vanishes).
- 🔴→✅ **A flowering plant at 5.6 m was a "person"** (YOLOX-Nano 0.3–0.69; the LiDAR ranged its leaves, person-sized). User: "can we use a bigger and more accurate model instead?" → `tools/compare_detectors.py` on all 894 frames of the recording:

  | model | ms / frame | plant detected | operator recall | people 9–11 m | unexplained ≥ 0.5 |
  |---|---|---|---|---|---|
  | Nano | 12.9 | **40–67 %** of its frames | 100 % | 0.65–0.80 | 10 |
  | Tiny | 14 | 0 % | 100 % | — | 2 |
  | **S** | 27.7–31.1 | **0 %** (even at 0.2) | 100 % | **0.86** | 3 |
  | M | 36.1 | 0 % | 100 % | 0.85–0.87 | 7 |

  Default engine → **YOLOX-S** (640 input, +4 ms CPU vs Nano); others via `engine_path:=`.

**Q58 — reid2 live test + fixes: a wall BEHIND the person ended the hold; no turning in place next to a person (2026-09-30)** ✅ *live-verified reid3 (12:49)* (6b536f4, release band in the Q58 reid3 bullet)
- *Recording `track_reid2_115215` (168 s, YOLOX-S):* the operator was **one id (#1) the whole session**; LiDAR holds beside the rover worked; search turned at 0.44–0.51 rad/s **with the slow zone occupied** (Q57 fix confirmed live); two losses, both re-identified (appearance 0.85 and 0.89, the second after 12 s of searching). **No plant detections with S.** Other ids: the operator's legs up close (unranged), a bystander in orange (#6–#8, real), one weak orange-object detection (0.39, never ranged → not selectable).
- 🔴→✅ **Both losses were the LiDAR cluster check, not the person leaving.** (1) "cluster 0.92 m wide": a separate object ~0.35 m beside them. (2) 3.5 s of "cluster 2.0–3.2 m wide" then LOST: the operator 1.5 m from the rover at +45° (out of camera view, clearly on the LiDAR), with a **wall 1–1.5 m behind them**. The old test counted EVERY point within 0.55 m (width) and 1.5 m (span ≤ 1.8 m) of the nearest point — connected or not — so anything standing near the person made them "too wide" (rule inherited from target_selector 1c8f06c).
  - *Fix (`lidar_cluster`):* only the **connected return** counts (scan neighbours ≤ 0.3 m apart, `cluster_link`, wraps at ±180°): body = its points within 0.55 m, ≤ 0.9 m wide; the whole connected return ≤ 1.8 m (a wall or car is one long return). *Pure connectivity was tried first and rejected:* in ids1 (118.8 s) and reid1 (110.1 s) the person is chained to a bush by a run of returns (1.0–1.6 m) that the body radius correctly cuts off.
  - *Replays (same bags, real clicks / operator selected at 10 s, re-ID on):* reid2 **2 losses → 0** (hold 287 → 407 frames; the 60–72 s hold ends in a camera match of the same track instead of a re-ID); ids1 ≥ old everywhere (hold 253 → 297 frames); reid1 identical (475 → 487). Remaining refusals are marginal 0.90–0.98 m bodies at 5+ m — 0.9 kept (wider starts merging two people shoulder to shoulder). `replay_tracker.py --why` prints each refusal.
  - *Also:* the old rule accepted the **end of a long wall** near the prediction as a person (span window 1.5 m ≤ 1.8); the new one refuses it (unit-tested). The hold now sorts the scan by bearing itself (a no-op on /scan_safety), so the rule does not depend on input order.
- 🔴→✅ **Safety — turning in place next to a person.** Footprint ±0.85 × ±0.58 m → **the corners sweep a 1.03 m circle** when the rover turns in place, and `/scan_safety` (range_min 1.0 m, mast self-returns) is **blind inside 1.0 m**, so the collision monitor cannot stop a turn into someone standing there. reid2's second search turned toward the operator 1.5 m away. New controller param **`min_turn_range` 1.6 m** (1.03 + a stride): while the target is closer — following, facing or searching — **no turn and no forward motion** (forward cut instantly); the search also uses where the person was **last seen**, never the extrapolated hint alone. *Found by the new `hug` scene:* the hint drifted to 1.9 m while A stood at 0.9 m → the first version swept; fixed, then 4/4 consecutive runs. Changes Q56's "near target: hold + face" to **hold** (no facing within 1.6 m).
- ⚠️ **OPEN — near-field blind zone of `/scan_safety` (both collision monitors, Nav2 and follow):** range_min 1.0 m in base_footprint hides everything within 1.0 m of the centre, while the footprint reaches 0.85 m ahead and 0.58 m to the side: the first 0.15 m in front of the bumper (inside the STOP polygon, which starts at x = 0.85) and the whole corner-sweep ring are invisible. Controller-side mitigation above covers follow-me's own turns toward/for the target only — not Nav2 in-place rotations, and not a second person. Proposed fix (needs the user's OK — safety config): record the raw cloud parked with nobody near, locate the mast/mount self-returns, replace range_min 1.0 with a self-filter box around the hull + mast (self_filter.py exists, unused) and range_min ~0.3.
- ✅ **Live reid3 (`track_reid3_124942`, 167 s, driven with FOLLOW; user: "looks good"):** the clicked person was one id (#2) from click to CLEAR; target from camera 638 / LiDAR hold 291 / near 3 messages. **Five episodes within 1.6 m** (down to 1.00 m, up to 162° off the nose): odometry yaw rate 0.5 s later median **0.000 rad/s**, forward ≤ 0.05 m/s — vs median 0.37 rad/s when the target was ≥ 1.6 m and > 10° off (turns normally when farther). One loss, by design ("on LiDAR alone > 5 s, 3.2 m away and something else within 1 m") → **re-identified 3 s later (0.89), following resumed by itself**.
  - 🔴→✅ *Found in reid3:* all 4 turning samples inside 1.6 m were at **1.57–1.60 m** — the operator stood ON the line and the rule toggled hold / turn (0.16–0.32 rad/s bursts; also the repeated "too close" log). **`min_turn_release` 0.2 m:** once holding, turn again only beyond 1.8 m. Controller 20/20 (1.7 m after 1.3 m: still holding); scenes hug/close/beside/main/reid 35/35.
  - ⓘ `test_follow_lidar main` (old selector path) failed 3× in a row on "no hop to the bystander … switch after 14 s", then passed 4× unchanged: it ran at load average ~9 (back-to-back scenes + the live T1/T2/T4 still up). The check times target_selector's hold expiry; nothing the controller publishes reaches the selector. Run timing-sensitive scene tests with the live stack down.
- *Tests:* unit 64/64 (physical scans: nearest return per 0.5° beam; the two new cluster cases fail on the old rule); controller 19/19 (1.3 m at 45°: no turn / no drive; 2.5 m: turns; lost at 1.3 m: search does not turn); click scenes 9 incl. new `hug` 53/53; old path test_follow_lidar 4/4 + 3/3; test_ui_server 10/10.

**Q59 — ArUco-marker following, same click-to-follow experience as people (2026-09-30)** 🟡 *built + tested offline end to end; live test pending (needs the marker's dictionary / id / size)* (3e6d196)
- *Request:* follow an ArUco marker mounted on anything (car, rover, …), "same thing as person following". Only a printed marker is available, no second vehicle → the operator carries it for the live tests.
- *Design:* markers are a second kind of target on the SAME interface, so follow_controller, the collision monitor, the 30 s search / resume and the 1.6 m no-turn rule (Q58) are reused unchanged.
  - `person_detector` also runs OpenCV ArUco (4.5.4 contrib, already installed) on the same frame → `/follow/marker_detections` (id + 4 corners, native pixels). `detect_people:=false` = markers only (YOLOX not loaded).
  - New `marker_tracker` (+ `marker_tracking.py`, no ROS): range / bearing from the marker's printed size (PnP, IPPE_SQUARE) or — where the LiDAR has a return at its bearing that agrees (0.3 m + 10 %) — the LiDAR's. Publishes `/follow/marker_tracks`, and for the SELECTED marker `/follow/track_id` / `/follow/target` / `/follow/target_source` (`marker` | `marker+lidar`) / `/follow/search_hint`.
  - **One selection at a time:** track id = 1 000 000 + marker id; each tracker clears its own selection when the other kind is clicked, so exactly one node publishes the target. Switching kind changes the track id → the controller disables, as for a different person.
  - Browser page: marker outline (cyan, `M7 3.2 m`), clickable; the smallest shape under the click wins (a marker on someone's back vs the person). `follow.launch.py marker_dictionary:= marker_ids:= marker_size:=` ("" = markers off, no CPU cost).
- *True positives only:* allowed-id list; confirmed after 3 sightings in 1 s; a sighting counts only if ≥ 10 px, PnP fit ≤ 2 px, within 75° of face-on, in range; motion gate (the same id elsewhere is ignored unless it persists 3 frames); after a loss 2 sightings to resume. No LiDAR hold in v1 (a car-sized return is what the person hold refuses as a wall): out of view = SEARCHING → the same id seen again resumes (the ID is the identity — exact re-identification).
- **Measured on real frames (reid3, 1665 frames, no marker in the scene):** detection **6.1 ms / frame** (p95 10.8). False decodes: DICT_4X4_50 in 4 frames, 4X4_250 in 13, ARUCO_ORIGINAL 1, **5X5 / 6X6 / 7X7: 0** — all single-frame, 10–19 px; **every one was rejected by geometry alone** (too small / poor fit / edge-on), none confirmed. → a 5×5 or 6×6 marker is the better choice if a new one is printed.
- 🔴→✅ **Range bias from OpenCV's corner refinement:** `CORNER_REFINE_SUBPIX` uses a fixed 5 px window — wider than a far marker's cells — and pulled the corners outward: range **3–7 % short at 3–5 m** (15 cm marker rendered through this rover's camera). Own cornerSubPix with window = half a cell (1–5 px): **+0.2 % at 1.2 m, +1 % at 3 m, +2 % at 5 m, +4–6 % at 6–7 m** (where the LiDAR range takes over). APRILTAG refinement was the most accurate (< 2 %) but missed half the markers beyond 4 m.
- 🔴→✅ **Heading from a flat marker is ambiguous:** a marker facing the camera at 3 m came out turned −25° (the two IPPE poses fit equally). Heading is now reported only when the second pose fits clearly worse, else NaN — exact at 2 m and for ≥ 45° at 3 m, unknown otherwise. Not used for control; position is unaffected.
- 🔴→✅ *Found by the end-to-end test (real nodes, rendered camera frames):* `marker_ids:=7` on a command line arrives as an INTEGER → marker_tracker crashed at start (dynamic typing); person_detector's shutdown assumed YOLOX was loaded. *By the live probe:* a static scene gave a DICT_4X4_1000 id in 26 % of frames → probe requires ≥ 50 % and ≥ 15 px.
- *Range limit at 640×480:* ~16 px per side (measured on renders, not the 15 px first assumed for any dictionary). **The user's marker: DICT_6X6_50, id 0, 17.5 cm black to black** (live probe 09-30: 63/63 frames at 114 px) → read 8/8 to 6.5 m, 5/8 at 7 m; range error +1.0 % at 3 m, +1.7 % at 4 m, +2.1 % at 5 m, +3 % at 6 m.
- ✅ **Live probe (first real marker data):** marker range − LiDAR range = **−0.13 m, spread 0.01 m** at 0.95 m — the user confirmed 17.5 cm, so the LiDAR was reading the BODY behind the hand-held sheet. → 🔴→✅ the tracker took the LiDAR's range whenever it "agreed" (0.3 m + 10 %), i.e. it would have reported what is behind / around the marker. Now the marker's own range while it is well resolved (≥ 20 px: ≤ 5.3 m for this marker, error ≤ 2 %); the LiDAR's only below 20 px and only if it agrees.
- 🔴→✅ **OpenCV 4.5.4 drops a marker with a narrow white margin at close range:** its too-close-candidates filter (`minMarkerDistanceRate` 0.05 = 14 % of the side) keeps the LARGER of two nearby squares — the outline of the white margin on a darker background — and drops the marker. A 6×6 marker with a one-cell margin (12.5 %) rendered at 1.2–2 m: **0/6 read**; 4×4 (16.7 %) unaffected. Found because the render disagreed with the live probe (which read the real marker at 0.95 m — its margin or background does not trigger it). Set to **0.03**: 6/6 at every distance, no duplicates (0.02 returns the same marker twice). Re-checked on 3332 real frames (reid2 + reid3): DICT_6X6_50 **0 raw decodes**, 6.3 ms / frame. Keep ≥ 2 cm of white around the black square anyway.
- *Probe:* waits up to `--wait` 180 s for a marker held steadily in view, then measures (2c9d245).
- *Tools:* `tools/marker_probe.py` (read-only; **waits up to `--wait` 180 s until a marker is held steadily in view**, beeps, then measures `--secs` 8 s — the operator has to walk to the rover with it; dictionary, id, size estimate from the LiDAR; `--size` compares marker range with LiDAR range) · `tools/marker_sim.py` (renderer: real K + upside-down mount) · `track_record.sh` records the marker topics.
- *Tests:* `test_marker_tracking` 37/37 (geometry + rules) · `test_marker_pipeline` 14/14, also `--people` with YOLOX-S + re-ID loaded (3.09 m / +11.3° vs true 3.06 m / +11.3° through the real detector + TF) · `test_marker_follow` follow 12/12 (only marker 7 ever the target, nothing guessed while hidden, rotation-only search, resumes by itself) + switch 11/11 (marker → person → marker, track ids never interleaved) · UI 15/15 (markers aligned to the frame; video keeps flowing if marker_tracker stops) · page script parsed and click geometry run under gjs · regression: persons 64/64, controller 20/20, click scenes 53/53, old path 4/4 + 3/3.

**Q60 — aruco1: first live tracking-only test of the marker (2026-09-30 14:24)** 🟡 *tracking verified live; one fault found and fixed offline on the recording; driven test pending* (c9f1e84)
- *Recording `track_aruco1_142439` (147 s, no FOLLOW; marker DICT_6X6_50 id 0, 17.5 cm, hand-held).* Read from 2.2 m to **~8.5 m** (13 px — further than the renders predicted: 194/199 frames at 7.2–8.5 m); only id 0 ever decoded; no sighting rejected; **no false marker** in the 40 s without it. Selection switching person #2 ↔ marker: `/follow/track_id` 2 → 1000000 → 2 → 1000000, each tracker clearing its own (never both). Marker turned away → SEARCHING → TRACKING when shown again (118–122 s). Range source: marker's own 566 sightings, LiDAR 235 (< 20 px).
- ✅ **Range check against the LiDAR (marker flat on the chest, 3.8–4.0 m, 100 frames):** marker − LiDAR = **−0.02…−0.03 m** (sd 0.03–0.06). Hand-held in front of the body at 2.2–4 m: −0.12 m (the body behind it, as in the probe). At 6–8.5 m: +0.06…+0.2 m (the +3–4 % far-range bias the renders predicted; the LiDAR's range is used there).
- ⓘ Carrier heading known in 5 / 801 sightings — as designed (ambiguous → NaN); not used for control.
- 🔴→✅ **At 2.2–4 m — the following distance — the marker was missed in 63 of 175 frames while the operator walked** (gaps up to 0.9 s live; the tracker drops to SEARCHING after 0.5 s, the controller after 0.6 s → the rover would have stopped and searched repeatedly while following). Standing at 4–7 m: 150/150.
  - *Cause (threshold images of the missed frames):* OpenCV's adaptive threshold calls a pixel dark if it is < local mean − 7. Beside the bright white margin the local mean is high, so the mid-dark shirt next to it also thresholds dark; the black square is then separated from that halo only by the **margin — 1.3–2.0 cm on this print (0.6–0.9 cells, 3–6 px)**, and a finger on the edge, a shadow or slight blur joins them → no square contour. Not blur and not a tuning of windows / polygon accuracy / error correction (all 84–89 of 175).
  - *Fix:* a **second detection pass** with threshold constant 25 and windows to 53 px, run only when a wanted id is missing: 87 → **170 of 175**, longest gap 44 → **3 frames** (JPEG frames of the recording, in sequence). Not as the only pass — constant 25 reads nothing beyond ~5 m (0 / 199 at 7–8.5 m; the default 171). A third pass (constant 15) added 3 far frames for +5 ms: rejected.
  - *Cost:* a pass is ~5.6 ms offline, ~10.5 ms live (aruco1 log). Throttled: every frame only while an id read within the last 2 s is missing; otherwise every 3rd frame → **+1.4 ms mean** with nothing in view (unthrottled +5.4 ms); a marker the first pass cannot read at all is still found within 0.3 s. `marker_ids` now also goes to person_detector (the ids being looked for).
  - *Checked:* 3332 marker-free frames (reid2 + reid3): **0 false decodes**. Replaying the recording through the tracker with the marker selected: SEARCHING episodes 5 → **1** (the deliberate turn-away at 88 s).
  - *Fixture:* `tools/fixtures/aruco_narrow_margin.npz` — 12 real 112 px crops the default pass cannot read (unit test: 0/12 → 12/12).
- ⚠ **Physical recommendation:** mount the marker on a flat WHITE board with **≥ 3 cm of white** around the black square (≥ 1.5 cells), nothing overlapping the margin. The second pass makes the present print usable; a proper margin removes the cause.
- 🔴→✅ *Test-harness races found while re-running UI tests under load (load average 6):* (1) "page disconnects → enable=false: never" — the page had just been silent 2.5 s and sent FOLLOW before its next heartbeat; the deadman (correctly, fail-safe) switched follow off 10 ms later, so nothing was left to switch off at the disconnect. Test now resumes heartbeats first and asserts follow was on. (2) "video keeps flowing" counted ≥ 8 frames in the last second; the harness's own timer gives 7–10 under load → now asserts no gap > 0.5 s (measured 0.12 s).
- ⓘ *Process note:* this session was interrupted mid-analysis; on resuming, part of the fix (second pass, `marker_ids` to the detector, the fixture) was already on disk uncommitted, written by the interrupted session in its last minute. Reviewed line by line, its numbers re-measured independently (171 / 175, 0 false) before building on it.
- *Tests:* marker unit 43/43 (fixture + throttle sequence) · pipeline 14/14 and 14/14 `--people` · marker scenes 23/23 · UI 15/15 ×5.

**Q61 — aruco2: first DRIVEN marker-following test (2026-09-30 15:00)** ✅ *following works live (user: "seems good for now")* · 🔴→✅ *CLEAR while following started a 29 s search — fixed, offline-tested* (322da9c)
- *Recording `track_aruco2_150009` (280 s, 14.9 m driven; marker DICT_6X6_50 id 0 hand-carried; people #8 / #10 followed in between).*
- ✅ **Walk-away:** the operator got 6.7 m ahead; the rover followed at 0.58–0.68 m/s and **stopped 3.09–3.15 m from the marker** (set 3.0 ± 0.25), bearing within ±2.5° on the straight.
- ✅ **Inside 1.6 m** (marker at 0.98–1.0 m, up to 17° off, 211–221 s): yaw rate **0.000 rad/s**, speed 0.00 — the Q58 no-turn rule holds for markers.
- ✅ **Detection while followed and in view:** 474 / 480 frames, longest gap 2 frames. ⓘ The second pass (Q60) was **not exercised** in this run: a single pass reads the same frames offline (evening light, no sun on the shirt) — it is proven on the aruco1 frames only. Marker pass live 12–19 ms per frame (detector still 10 Hz: 2784 frames / 280 s). Range from the marker 633 sightings, LiDAR 59.
- ✅ **Searches (7) were all real losses of sight, each recovered:** walking round the rover's side at 2.3 m (rover turned 135°, found again after 8 s); marker cut by the image edge while walking sideways at ~2 m (5 ×, 0.6–1.9 s each — the rover's 0.5 rad/s cannot keep a marker centred on someone walking past at 2 m); the deliberate turn-away (below). No miss of a fully visible marker started a search.
- ✅ Marker ↔ person switching: the controller disabled on every change (`target changed (#1000000 -> #8)` …), FOLLOW needed again each time.
- 🔴→✅ **CLEAR while following → the rover turned in place for 29 s (736°, two full turns)** until STOP was pressed (222–251 s). CLEAR only removed the selection; with no target the controller saw a *loss* and ran its 30 s search. Same on the person path. *Fix (follow_controller):* it now listens to `/follow/select`: **−1, or an id other than the one followed, ends the session at once** (`disabled: selection cleared` / `target changed`), also during a search and before the newly clicked target has produced anything; clicking the target already followed changes nothing. Controller test 26/26 (incl. CLEAR during a search: turning stops); marker scene `follow` now ends with CLEAR → disabled within 0.3 s, no motion after.
- ⚠ **Turn-away test (95–113 s): the marker was hidden with the operator standing 3.3 m in front; the rover faced the spot for 3 s, then swept 333°** and only saw the marker again when it came round (18 s). By design (v1 has no LiDAR hold for markers), but it is the wrong behaviour for a marker on a vehicle that turns: the carrier is still there. → **next: hold the carrier on the LiDAR** (the connected return at the marker's last position, continuity-gated) so the target survives the marker being out of view, as people's does.
- ⓘ **Speed:** 0.74 m/s peak at the end of the 0 → 0.6 ramp (above 0.66 for 2.4 s, above 0.70 for 0.8 s; cruise 0.58–0.63). FCU speed-loop overshoot; the 09-26 validation saw 0.60–0.63. The command was not recorded → `track_record.sh` now records `/cmd_vel_follow_raw`, `/cmd_vel_follow`, `/follow/enable`. FCU tuning is the user's.
- ⓘ **Odometry slides backward while the rover stands** (10 episodes, 0.03–0.19 m, 1–2 s, after turns in place / stops / follow switching off; twist down to −0.16 m/s). LiDAR scan match over the same intervals: 0.00, 0.00, −0.06 m where odometry says −0.11, −0.12, −0.20 (parked control: 0.00 both) → **estimator slide, not motion**; the controller never commands reverse. Same family as P10 (ZUPT covers neutral PWM ≥ 1 s only). No effect on following (targets are in base_footprint); noted for the SLAM / Nav2 side.
- *Test-harness artefact fixed:* `test_marker_follow` logged controller states during its 6 s warm-up on the previous clock → a new "never disabled between FOLLOW and CLEAR" check failed on entries stamped 4.5–5.6 s. Log cleared at scene start.
- *Tests:* controller 26/26 · marker scenes 25/25 ×3 · click scenes 53/53 · old path 4/4 + 3/3 · UI 15/15.
- *Docs:* `COMMANDS.md` added to the repo (user request 09-30): every build / bring-up / stop / navigation / SLAM / following / recording / analysis / debugging / test command, one line each; linked from README. To be kept current with every new or changed command.

**Q62 — Follow page on the SIYI MK32 handset (2026-10-02)** 🟡 *built + tested offline; live test on the MK32 pending*
- *Request:* the click-to-follow interface on the SIYI MK32 instead of a phone / laptop, very low-latency video; the Jetson is wired to the SIYI air unit's LAN.
- *Checked (read-only):* Jetson `enP8p1s0` = **192.168.144.100**/24, 100 Mb/s. On the subnet: **.11** air unit (0.3 ms), **.12** ground unit (1.9 ms, across the radio), **.20** the MK32's **Android** system (TCP 5555 = Android network debugging open) **4.5 ms mean / 7.8 max, 8.1 ms with 1400-byte packets, 0 % loss** (bench distance), .33 a local wired device (0.2 ms, probably the LiDAR). `ui_server` already listens on 0.0.0.0:8080, no firewall → **the page is reachable on the handset as is: `http://192.168.144.100:8080`**. Video is 29 KB/frame median (41 max) ≈ 2.3 Mb/s at 10 fps.
- *Rejected:* H.264 into SIYI's own FPV app — hardware decode on the handset, but the Orin Nano has no hardware encoder (software x264 = +54 % CPU, Q54) and the app cannot click-to-select.
- *Built (ui_server + page):*
  - **Handset layout** on touch screens (`?handset=1/0` forces): full-height video, STOP / FOLLOW / CLEAR / LITE / FULL as a column of big buttons, no zoom or scroll, fullscreen + landscape lock on FULL. **Forgiving taps:** a tap that hits nothing takes the nearest person / marker within ~28 screen px.
  - **Older Android WebViews:** no DOMMatrix / DOMPoint (own 2D affine; checked against the old mapping for all four rotations under gjs), `<img>` decode when `createImageBitmap` is missing.
  - **LITE video** per page (`/ws?video=lite`, or the button): JPEG decoded straight at half size and re-encoded — **320×240, ~10 KB (34 %), 2.4 ms**, once per frame shared by all LITE pages, in a worker thread. Only for a slow link: at short range the radio carries a full frame in about the time the re-encode takes.
- 🔴→✅ **Deadman belonged to ANY page:** it watched the newest heartbeat of all connected pages, so a laptop page left open kept follow alive after the operator's handset lost its radio link. Now the page that pressed FOLLOW owns the deadman: silent 1.5 s or disconnected → follow off, whatever else is connected (tests: 1.3–1.5 s / 0.01–0.14 s with another page heartbeating; the old code: never).
- *Tool:* `tools/ui_latency.py` — connects like a page (no heartbeats, no commands) and splits each frame's delay: camera → sent (Jetson) and sent → here (network). Smoke-tested against a real ui_server in an isolated domain.
- *Tests:* `test_ui_server` 21/21 ×3 (LITE 320×240 beside full 640×480 on the same frames; switching by message; deadman tied to the FOLLOW page; page has no DOMMatrix dependency).
- *Next:* (A) open the page on the MK32, read age / fps / RTT, and `ui_latency.py` on the Jetson for the camera → sent baseline; (C) trim the Jetson-side delay where it is measured to be (marker detection and the UI JPEG currently run after YOLOX, before the frame can leave); (D) optionally a physical MK32 switch for follow STOP / FOLLOW through its RC channel → `/mavros/rc/in`.

**Q63 — MK32 live: the page works; latency split and VIDEO FIRST (2026-10-02)** ✅ *live-verified on the MK32 (15206ff)*
- ✅ **Live on the MK32 (user):** page renders, **age ~150 ms, 10.0 fps, RTT 10 ms**. `ui_latency.py` on the Jetson: **camera → sent 100.5 ms** median (p95 111), sent → here 1.7 ms, 28 KB/frame (2.3 Mb/s). → ~50 ms is radio + the handset's decode / draw; **~100 ms is on the Jetson**.
- 🔴→✅ *Seen in the user's photo:* the **desktop layout** came up (buttons in a row, keyboard hints): the MK32's browser does not report `pointer: coarse` → handset = `ontouchstart` / `maxTouchPoints` / `any-pointer: coarse`. Status items ran together ("connectedage 152 msfps"): its browser **predates flex `gap`** → spacing by margins (no `gap` / `inset` / `aspect-ratio`; checked).
- *Where the 100 ms goes (read-only probe of each stage, capture stamp → received, medians):* camera frame 40 ms · detections 92–94 (YOLOX-S 38 + markers 12, **in series**) · tracks 97 · UI JPEG 99 (encoded only after detection) · sent ~100 (held until its boxes). **55 ms of the video's delay was detection it does not need.**
- *Fix — VIDEO FIRST:* `person_detector` encodes + publishes the operator's JPEG **and** runs ArUco detection in a worker thread **while YOLOX runs** (cv2 releases the GIL: 40.6 → 28.2 ms for both, measured); `ui_server` sends each frame **the moment it arrives** and its boxes as a separate `{"type": "boxes", "stamp": …}` text message when both trackers have that frame; the page draws a frame with the previous boxes and redraws it with its own when they arrive (boxes always match their frame; taps resolve against what is drawn). Expected: video out at ~45 ms instead of ~100 (→ ~95 ms on the handset), boxes ~50 ms after their frame.
- *Handset UI redesign:* video at full height on the left with a status overlay (link dot + age / fps / RTT colour-coded, people / markers, target); right column: a big state tile (OFF / FOLLOWING / SEARCHING / STOPPED + reason), STOP (largest), FOLLOW, CLEAR, LITE | FULL.
- ✅ **Live after the change (T4 restarted, user on the MK32):** `ui_latency.py` **camera → sent 51.5 ms** (p95 55.5; was 100.5), sent → here 2.4 ms, **camera → boxes 97.2 ms** (p95 107; unchanged, as expected), 27 KB/frame, 10.0 fps, a boxes message for every frame. **Handset age ~100 ms (was ~150)**, 10 fps, RTT 10 ms; person detection and selection work from the handset (user). The frame leaving at ~52 ms while detections take ~92 ms is the live proof of the ordering the skipped `--people` check would have tested. Left on the Jetson: the camera (~40 ms capture → ROS) + JPEG (~4 ms); on the handset: ~45 ms decode / draw / display (LITE may shave some at range).
- *Tests:* `test_ui_server` 22/22 ×3 (every frame gets its own boxes message; a frame leaves < 15 ms after its image arrives — 3.5–5 ms; boxes < 15 ms after the tracks); `test_marker_pipeline` 15/15 (the detector's JPEG for every processed frame, same stamp as its detections; the "JPEG before detections with YOLOX" check runs with `--people` only — not run: 1.2 GB free with the live stack up, a second detector risked an OOM kill of live nodes); page script parsed + transform checked under gjs.

**Q64 — Reports and learnings now in the repo (2026-10-02)** ✅
- User: "include our all reports and learnings in github, and keep it updated". This report, `ardupilot_config_checklist.md`, `next_session_prompt.md`, the monitoring tools and the 09-17 backup moved to `ros2_ws/docs/reports/` (copied byte-identical, then the originals replaced by symlinks — `/home/jetson/dagger_reports/…` still resolves). Learnings: a copy of Claude Code's project memory in `docs/learnings/claude-memory/`, refreshed by `tools/sync_learnings.sh`; `docs/README.md` indexes it all. Checked first: no credentials in any of it (only a note that a sudo rule lets nvidia run as jetson).
- *Rule from now:* edit the files under `docs/reports/` (not through the symlink); at each session wrap-up sync learnings, commit, push.

**Q65 — ⚠️ Near-field blind zone (P13) MEASURED: a person beside the rover is invisible to both collision monitors (2026-10-02)** ✅ *fixed (`near_field:=true`): disarmed bench + armed Nav2 turn into a box verified live*
- *Captures (parked, disarmed, live T1+T2):* `body_empty_132834` (39 s, nothing within 1.6 m) and `body_walk_132959` (119 s, the operator upright ~10 cm off the hull at 8 spots: front centre, left front/middle/rear, rear centre, right rear/middle/front). Cross-checked against `slam_lane3_164840` (09-26, 41 m driven — what stays fixed in base_footprint while driving is the rover).
- *The "mast self-returns" are the rover's top frame:* a U — side rails y ≈ ±0.51 from x ≈ 0 back to −0.84, rear rail x ≈ −0.84 — at z 0.81–1.11, plus two posts at (0.31–0.41, ±0.30), z 1.05–1.11; ~1,100–1,300 points per cloud. Extent x −0.85…+0.42, y ±0.56 (raw, both dates): **entirely inside the hull outline**; nothing new since 09-26. The rails occupy every bearing from ±90° to 180°, so a scan-level filter would blind the rear half — the filter must act on the cloud.
- *Box x −0.90…0.50, y ±0.60, all heights* (≥ 4 cm margin; front edge 0.45 → 0.50 because the posts reach 0.42) + range_min 0.3: empty capture **0 returns within 1.6 m** in the 0.30–1.60 band over 387 clouds; walk capture 0 with nobody near.
- *Operator per spot, rays within the 1.03 m corner sweep (median per scan), today's `/scan_safety` → box + range_min 0.3:* front centre 27 → 65 · left front 18 → 71 · **left middle 0 → 87** · left rear 24 → 41 · rear centre 39 → 58 · right rear 32 → 48 · **right middle 0 → 81** · right front 9 → 67. Beside the sides the operator was **completely invisible today** — an in-place turn would sweep a corner into them unchecked. Heights seen in the ring: 0.93–1.32 m at the front half, 0.78–1.46 m at the rear (the frame has gaps; the rear sees lower than the beam-geometry estimate of ~1.15 m). Children, crouching people and low objects stay invisible (sensor limit).
- *Shadowing found:* at the front centre the operator's upper body was over the front deck (x 0.80, ~2,100 points/cloud inside the footprint). With the near field visible those become each bearing's nearest return, so stop-zone rays drop **29 → 8** (still > 3) while 58 rays land inside the footprint (approach → 0). Proposed with the fix: PolygonStop inner edge x 0.85 → 0.50 in both monitors, so anything over the front deck stops the rover without relying on the approach polygon's footprint topic (fails open silently when absent, see the follow-monitor yaml).
- *A numpy model of p2l (range_min 1.0, no box) reproduces the recorded `/scan_safety`:* 723 bins, median 100 % of bins within 5 mm, worst scan 99.72 %.
- **Built (user OK'd build + stop-zone change):** `safety_scan`, a C++ node in the new package **`dagger_perception`** (ament_cmake), replaces p2l_safety when T2 runs with **`near_field:=true`** (default `false` = p2l_safety, range_min 1.0 — one-word revert). Cloud → drop the body box (x −0.90…0.50, y ±0.60, all heights) → same band / 723 bins / range_max 20 / inf as p2l → `/scan_safety`, range_min 0.3, best-effort like p2l. Mount transform read once from `/tf_static` (`tf2::BufferCore`, no `/tf` listener); permanent subscription, so no lazy-subscribe wedge. `scan_safety_watchdog` follows the switch (process pattern). Why not the alternatives: a scan-level box filter would blind the rear half (above); Python just *receiving* this cloud costs ~10 % of a core (system_ready A/B).
  - *Exact equivalence:* with the box off and range_min 1.0 it reproduces the live `/scan_safety` **bin for bin on all 1,572 scans** of the two captures. First build: 93 of 1.1 M bins differed (≤ 2 per scan, 4.6 / 10 m) — a static point on a bin edge changed bins because range/bearing were computed in double; p2l's unqualified `hypot`/`atan2` on floats use the float overloads. Float math → 0 differences.
  - *CPU (10 Hz replay of the walk capture, isolated domain, by PID):* **safety_scan 6.3 % vs pointcloud_to_laserscan 13.0 %** of one core.
  - *Tests:* `tools/test_safety_scan.py` **25/25** (domain 78): 12 synthetic (body frame + posts gone, 3 points just outside the box kept, person beside the side: 29 rays vs p2l 0, band/range edges, nearest per bearing, bin index, NaN, empty), equivalence, parked capture 0 returns < 1.6 m, walk capture all 8 spots ≥ 41 rays and 0 with nobody near. Consumers: person tracking 64/64, marker tracking 43/43, follow controller 26/26 at range_min 1.0 **and** 26/26 at 0.3 (`SCAN_RANGE_MIN`, new option of test_follow_controller / test_click_follow, also used by test_marker_follow). Stack down: click scenes 53/53 at 1.0 and 53/53 at 0.3 (hug puts the target at 0.9 m: its returns now reach the tracker, whose own min_range 1.0 keeps the behaviour), marker scenes 25/25 + 25/25, marker pipeline 15/15, UI server 22/22, old follow path 4/4 + 3/3.
  - *Prediction for the disarmed bench (walk-capture scans through a model of the stop zone + approach polygon, footprint ±0.86 × ±0.59):* turning left (wz +0.45) — front centre / left front **STOP** (stop zone from x 0.50), left middle ×0.40, right middle and right rear ×0.07, left rear and right front **CLEAR** (the hull swings away); turning right mirrors it. Nobody near: CLEAR.
- **PolygonStop inner edge x 0.85 → 0.50** in both monitors (collision_monitor.yaml, collision_monitor_follow.yaml; user OK) — applies at the next T2 start, harmless with near_field off (that region is inside range_min 1.0). `follow_solo_gate.py` uses the same zone.
- `tools/follow_gate_test.py`: `--path nav|follow` (the Nav2 monitor can now be bench-tested too), `--vx 0 --wz …` judges the turn rate, `--expect-clear` for the false-trigger run, nearest return in any direction. Its SLOW check is now Nav2-only — the follow monitor has had no slow polygon since the slow zone moved into the controller (Q57). Disarmed, an in-place turn is only *scaled* (time-to-contact / 1.5 s) by the approach polygon unless the person is in the stop zone; the stop distance under a real turn is for the live test.
- Comments brought up to date: follow_controller `min_turn_range` (kept at 1.6 as the second layer), person_tracking near mode, monitor yaml headers.
- **Live, T2 `near_field:=true` (user OK), DISARMED bench, bag `pf_bench_nf_144406`:** safety_scan up in 0.1 s (mount from /tf_static = URDF), /scan_safety 9.95 Hz best-effort, parked 0 rays < 1.6 m; Nav2 PolygonStop from x 0.50 confirmed by parameter read (collision_monitor_follow belongs to T4, not running — same yaml change).
  - *Step 1 — false triggers:* `follow_gate_test --path nav --vx 0 --wz 0.45 -t 600 --expect-clear`, nobody within 3 m: **CLEAR 100 % of 600 s** (PASS).
  - *Step 2 — turn left (wz 0.45), operator 0–15 cm off the hull:* left front **STOP**; right middle (0.60–0.66 m from centre) **×0.07**; right rear (−138°) **0–×0.07**; right front, stood 2–5 cm off so points fell inside the footprint → **STOP**; left side a little behind the middle (+105°) ×0.7–0.9 (the hull needs ~1.1–1.4 s to reach that spot). Against the model on the live scans: median |measured − model| 0.00; the monitor is one 0.1 s simulation step MORE conservative (model ×0.13 → measured ×0.07; ×0.07 → 0). The only larger mismatches are ×0.40 while walking past the front: PolygonSlow also scales turns (Humble scales the whole twist), not in the model. With range_min 1.0 these positions gave 0 rays (walk capture) — the monitor would have passed the turn untouched.
  - *Step 3 — forward 0.3 m/s, walk-in:* SLOW from 2.27 m ahead, STOP from 1.31 m (zone edge 1.45, 0.5 s sampling), STOP held 13.5 s from 0.94 down to **0.56 m** (leaning over the front deck), SLOW again at 1.80, CLEAR at 2.80.
  - ⚠️ *Two limits found while placing the object for the armed turn test (live scans, model of the monitor):* (1) **`max_points: 3`** — a monitor zone triggers only on more than 3 rays, so anything narrower than ~3 cm at 1 m is ignored: a 2-ray stand edge 12 cm off the hull would have been hit before the monitor saw the object. (2) **The approach polygon scales the turn by time-to-contact and needs > 3 rays inside the simulated hull**, so real objects (edges, bumps) touch first: for a ~50 cm object 40 cm off the left side the model gives first touch at 24°, monitor contact at 27.5°, stop with **0–9 cm** to spare depending on the FCU's turn deadband; a 15 cm static hull-margin stop ring only reaches ~4–8 cm with an assumed 0.3 s latency. The margin fix needs the measured deadband and latency → armed turn test with a soft object first.
  - ⓘ *Phantom returns from the rover's edges (mixed pixels):* placing the box revealed a thin "object" 11–25 cm off the left side, 0.94–0.98 m high, that the operator confirmed was not there. All 144 points came from ONE ring (−12.4°), a 2° azimuth window, ranges 0.67–0.85 m along that beam — the beam clips the top edge of the left rail and returns a range between the rail (~0.5 m) and the box behind it (~1.0 m). Only appears with a real object within ~0.5 m beyond an edge, always NEARER than that object (conservative), behind the posts (x ≤ 0.45, outside the stop zone). Picture: `docs/reports/near_field_object_2026-10-02.png`.
  - ✅ **Armed Nav2 turn into a box (bag `pf_turn_obj_nf_155948`, operator-run `send_goal.py 0 --dyaw 90`, GUIDED):** box ~50 cm wide, face 40–42 cm off the left side beside the mast (outside the stop and slow zones — only the approach polygon could act). Controller asked 0.45 rad/s throughout; the monitor passed ×0.64 at once (box already within 1.5 s), then ×0.93 → 0.53 → 0.47 → 0.40 → 0.33 → **0 at 22°**. Gyro peak 0.35 rad/s, stopped within 0.25 s of the zero, **no coasting**; **13 cm** left between the hull and the box (13.4 cm to the 4th-nearest ray), held 16 s until the operator cancelled. Model predicted 17–25° and 0–9 cm — better live because the FCU barely executes small turn rates (0.15 rad/s commanded → 0.03–0.08 actual). Twice the output flickered back to ×0.33 for one step (edge rays), ±0.6° twitches (< 1 cm at the corner). **No hull-margin stop ring needed on this evidence**; revisit if a drive shows otherwise.
  - *Stop path:* in Humble the monitor publishes zeros for `stop_pub_timeout` 2 s and then goes silent; during all 8 silent windows the FCU setpoint (`cmd_vel_safety`, 20 Hz) was **exactly 0**. FCU never armed.

**Q66 — Detour test, tall control box (2026-10-03, bags `detour_tall_110439`, `detour_tall2_114209`)** ✅ *detour OK twice; run 1 stopped by the collision monitor at a bad goal, run 2 succeeded*
- *Site:* test lane, wall on the right, steps on the left (P18). Rover 1.63 m from the wall; tall box (~55 cm wide) on its line, near face 7.6 m ahead; the right side of the box closed by a wall-side object (6.9–7.2 m, 0.6 m right of the line) → only a left detour possible. Goal `send_goal.py 12 --left 0.2` (Claude's choice).
- *Detour:* Theta* bent left from the start (tall box in the costmap from the beginning); the rover passed the box with its centre 1.1 m left of the line → **hull 0.2–0.4 m from the box** (Theta* hugs obstacles; inflation 0.8 / scaling 6).
- *Stop:* returning toward the line, SLOW at 76 s, **STOP at 82 s, bumper 0.54 m from a plant/sign** (thin stem, top 1.3–1.8 m) standing 12.0–12.5 m along, ~0.3 m right of the line — i.e. on the goal. The costmap had it from t = 0 (45–54 cells) but the planner checks only the robot CENTRE, so a goal 0.57 m from the plant (inside the hull's reach, outside the inscribed radius) was accepted. Nav2 kept commanding 0.4 m/s for 30 s; the monitor held zero (stop_pub_timeout → silent → setpoint 0). Safety layer correct; the goal was wrong.
- ✅ **Run 2 (`detour_tall2_114209`, after the Q67 power-restart, T2 restarted with the new guards):** rover 2.41 m from the wall; box (~55 cm wide, ~0.65 m tall) near face 5.96 m ahead on the line; plant found by its canopy (z 1.4–1.9) 14–15 m ahead, 0.8–1.1 m right → goal `send_goal.py 11 --left 0.2` (stop zone 1.5 m short of the plant). Nav2 **SUCCEEDED** (status now recorded); stopped 10.70 m along, +0.29 m. **Left detour**, centre 0.79–0.87 m left of the line beside the box; **clearance ~0.21 m** (scan, box re-seen behind the right rear corner at 40–42 s; beside the hull the 0.65 m box is below the LiDAR's near-field view, ~0.9 m). PolygonSlow fired for 8 s (3.3–4.5 m along, 0.16 m/s) — the box sat inside the slow zone while the rover angled past it. No other intervention.
  - ⓘ *Theta\* hugs obstacles:* 0.2 m clearance at 0.4 m/s with inflation 0.8 / scaling 6. Tuning candidate for the detour work (wider high-cost band or `w_traversal_cost`), not done yet.
- *Lessons:* (1) when choosing a goal, check the hull footprint + stop zone AT the goal for obstacles, not just the line up to it; (2) a thin stem with a canopy at 1.3–1.8 m is barely seen from 12 m (0–2 rays) — fill-in only inside ~5 m; (3) `ros2 bag record` skips `_action` topics unless `--include-hidden-topics` (fixed in `detour_record.sh`); (4) the goal was sent while disarmed — the rover waited until armed (fine, but the run's timing starts at arming).

**Q67 — 🔴 FCU EKF DIVERGED after the GNSS heading dropped out; preflight still PASSED (2026-10-03, ~11:10–11:20)** ✅ *recovered (full power-restart) + guards built*
- *Found by:* after the tall run, the operator repositioned the rover ~0.7 m; our odometry then put the run's start **88 m away** (T1/T2 never restarted). Parked and disarmed: `/mavros/local_position/odom` moved **109 m in 10.7 s**, `global_position/global` spread **429 m** in 10 s, `velocity_body` up to **198 m/s** (median body speed **253 m/s** a few minutes later). Our fused `/odometry/filtered` stayed frozen while parked (ZUPT on neutral PWM) but integrates the FCU's velocity whenever the tracks move — the 88 m jump.
- *Cause:* the dual-antenna heading is valid in only **8 %** of `gps1/raw` messages (yaw = 65535 otherwise; values 51.4–52.1° when present), and `/gps/heading_imu` publishes 8 msgs / 20 s; `EK3_SRC1_YAW=2` = GPS yaw only, no fallback → the EKF lost yaw and diverged. Fix itself was fine (3D, 24–26 sats, eph ~0.65 m, receiver velocity 0). The lane (wall + plants) likely obstructs the moving-baseline solution; whether it worked at 10:50 is unknown (no GNSS topics recorded), but the tall run at 11:04 drove normally.
- ⚠️ **`tools/preflight.py` PASSED with the EKF diverged:** it checks topic rates (`/gps/heading_imu` minimum 0 Hz) and prints fix/sats/yaw, but has no check of heading availability or of the FCU's velocity/position sanity while parked. In GUIDED the FCU's speed loop uses this velocity — arming here would be dangerous.
- *FCU's own messages (T1 console, restart 11:22:45):* right after connecting, `FCU: AHRS: DCM active` ↔ `AHRS: EKF3 active` flipping every ~0.1 s, again 2 s later — EKF3 unhealthy, the AHRS falling back to DCM. These STATUSTEXTs go to the T1 console only (not to `~/.ros/log`). `system_ready` meanwhile printed "ALL SYSTEMS READY … safe to drive" — it checks fix and satellite count only.
- *Recovery:* a full power-restart of the rover (operator) — heading back to 100 % valid, FCU parked speed 0.014 m/s. A T1 restart alone (11:20, 11:22) still showed the AHRS flipping.
- **Guards built (operator OK):** `preflight.py` judges the GNSS heading over the whole window (FAIL < 90 % valid; it only looked at the last message, which happened to be valid) and adds **FCU estimator while parked** (tracks at neutral: FAIL if speed > 0.5 m/s or position moves > 2 m in the window; healthy 0.00 m/s / 0.09 m), `/gps/heading_imu` minimum 0 → 2 Hz (3.6–5 Hz healthy, 0.4 Hz in the fault); `fcu_odom_relay` **`max_plausible_speed` 2.0 m/s** in both relay instances — faster FCU samples are dropped (ERROR logged), the parked ZUPT zero still published; `system_ready` now says "SENSORS UP → run preflight before driving" instead of "safe to drive". `tools/test_estimator_checks.py` 9/9 against a fake FCU replaying the fault (8 % heading, 250 m/s parked) and a healthy one; live preflight PASS with the new checks.
- *Was:* preflight FAIL if the GNSS heading is valid in < 90 % of messages (with EK3_SRC1_YAW=2) or the FCU's body speed / local position moves while parked; `fcu_odom_relay` rejects implausible FCU velocities (> 2 m/s on a 0.6 m/s rover) so our odometry cannot jump. FCU side (operator's call): reboot the FCU once the heading is back; consider `EK3_SRC1_YAW=3` (GPS yaw with compass fallback).

**Q68 — ⚠️ LIVE: Nav2 planned around a low box, then DROVE OVER IT once the box was erased (P14 confirmed, 2026-10-03, bag `detour_low_114923`)** ✅ *fixed: low-obstacle memory v2 (`low_obstacle_memory:=true`), live-verified*
- *Setup:* same lane and start as Q66 run 2 (2.58 m from the wall); cardboard box, top 26 cm, near face **7.24 m** ahead, slightly right of centre (top edge seen y −0.29…+0.02); `send_goal.py 11`. Nav2 SUCCEEDED — and the operator saw the right track run over the box's left edge.
- *Timeline (start frame):* the −7° beam caught the box's top edge at 7.2 m — **in the global costmap from t = 0** (4–5 lethal cells; not in Q30's visibility windows, which assumed 20 cm). Plans detoured **+0.74…+0.90 m** left at the box (≈ the tall box's +0.8). Box **faded** at 2.6 m along (1 cell, box 4.6 m ahead), **erased by 3.4 m along (box 3.8 m ahead)**; the local costmap held it only briefly. At **4.1 m along (box 3.1 m ahead) the next 1 Hz replan went straight through it** (−0.24…+0.10); the rover, then +0.2 m left, passed the box with its centre **+0.35…+0.40** → hull right edge ~0.2 m inside the box. One early plan (23.3 s) also went straight for a moment (box cells flickering). The collision monitor never saw it (26 cm < the 0.30 m safety band, and below the near-field view).
- *Comparison:* tall box (Q66 run 2, never erased) passed with centre +0.8 m, ~0.21 m clearance.
- *Conclusion:* P14 is real — a low obstacle the planner has seen and committed to avoid is driven over. Building the low-obstacle memory v2 (plan below); this bag is the replay test.
- **Built — low-obstacle memory v2:** C++ **`low_scan`** (dagger_perception) on `/patchworkpp/nonground`, one pass, same /tf_static + body box approach as safety_scan: per bearing nearest low (12–35 cm) and tall (40–160 cm) return; a low return with something tall within **±2 bearings** nearer than low + 0.5 m → NaN (Q32's blobs: a torso one ray away); low-only → kept only once its **15 cm odom cell (+ 8 neighbours) was hit in ≥ 3 scans spanning ≥ 0.5 s within 2 s** (first version — see the walk-by below for the final 0.5 m / all-tall-points veto and 5 hits / 1.0 s) (odometry from `/odometry/filtered`; none → nothing confirmed); tall-only → NaN; neither → inf. Feeds the existing `low_obstacle_layer` (clears only beyond 4 m) → `/scan_low`. Replaces p2l_low_all + p2l_tall + the Python `low_scan_filter` (deleted). `low_obstacle_memory:=true` (T2, forwarded by nav2_gps) now also ENABLES the layer through the overlay `params/low_obstacle_memory.yaml` (v1 needed a hand edit of `enabled`). **5.4 % of one core** at 10 Hz.
  - *`min_span` added after a test failure:* with only "3 hits in the 3×3 neighbourhood", a low-only walker at 1.4 m/s was confirmed in 17 of 20 scans (3 consecutive scans still land in one 45 cm neighbourhood); requiring the hits to span ≥ 0.5 s → 0 of 20. Cost: a box is confirmed after 0.5 s (6th scan).
  - *Tests (`tools/test_low_scan.py`, domain 80) 12/12:* synthetic — box confirmed from the 6th scan, also in front of a wall 4 m behind and while the rover drives at 0.4 m/s; person standing still (torso 1–2 bearings off) never; walker never; no odometry never; empty → inf, tall-only → NaN. **Replay of `detour_low_114923`** through low_scan + a model of the layer (0.1 m grid, marks < 8 m, clears beyond 4 m, footprint clearing): **the box is remembered from t = 0 (7.24 m ahead) until the bumper reaches it** (≥ 3 cells throughout); afterwards 11–13 cells were swept by the footprint — exactly the track over its corner; **0 other remembered cells in the driven lane**. Replays for false positives — remembered cells the rover then drove over: **0** on `detour_tall2_114209` (the tall box never enters the memory), **0** on the 41 m mapping drive `slam_lane3_164840`; the hundreds of remembered cells in each are low structure beside the lane (wall side, plants/steps side).
  - 🔴→✅ **Live walk-by (bag `detour_walkby_121626`, parked, 138 s, one walker crossing at ~3 m, along the left side, standing and crouching ~3.9 m ahead):** the costmap lane went 0 → **7 lethal cells left behind** at 3.6–4.1 m ahead where the walker stood/crouched — nothing there after they left (0 non-ground points). Two weaknesses, both fixed and replay-verified:
    1. *Tall check by bearing count:* at 4 m ±2 bearings = 7 cm; a crouching person's feet/knees 0.35 m beside the torso escaped → **veto now metric: any tall point within 0.5 m (2D)**.
    2. *Nearest tall return per bearing only:* with something tall nearer on those bearings, the walker's own torso right beside their feet (0.00 m in the raw cloud) was never compared → **veto now against ALL tall points**, via a 0.25 m spatial hash per cloud.
    3. *Persistence 3 hits / 0.5 s* still remembered feet lingering 0.5–0.8 s and 7 brief transients beside the 41 m drive (2–10 clouds each, never consistent across passes) → **5 hits spanning ≥ 1.0 s**. Replays with the final settings: low-box run — box remembered from 7.24 m to the bumper, 0 other cells near the later path; walk-by — **0** cells left in front; tall run — 0; 41 m drive — 1 cell near the path, seen at the same spot in two passes 26 s apart (17 s, 43 s): a real low object. `test_low_scan.py` 13/13 (new case: crouching person). Cost: a box is confirmed ~1 s after first sight (still ~6.8 m out for this box).
  - ✅ **Live walk-by 2 (`detour_walkby2_124019`, T2 restarted with the final low_scan: min_hits 5, min_span 1.0, tall_radius 0.5):** same routine (crossing at ~3 m, along the left side, standing + crouching ~3 m ahead) → **0 confirmed low rays in the lane area in all 1,111 scans**; costmap lane 0 → up to 21 cells while the walker was there (normal layer, legs) → **0 at the end**.
  - ✅ **Live low-box run with memory on (`detour_low_mem_124652`):** box (top 26 cm) near face 5.8 m ahead, slightly left of centre; confirmed by low_scan while still parked (7–8 rays/scan) and in the global costmap (8 cells) before arming. `send_goal.py 11` → **SUCCEEDED**, stopped 10.57 m along on the line. The planner took the wall side (box left of centre); **the box stayed in the global costmap at 8–12 cells throughout — 4 m ahead, beside, behind** (baseline: erased 3.8 m out) and in the local costmap from 4 m; the path bent smoothly, no snap back; centre ~1.0 m right of the line beside the box → **hull ~0.28 m from the box, no contact** (operator: "it did well"). PolygonSlow held 0.15 m/s for 12 s (29.5–41.5 s) while the rover angled 13° toward the wall past the wall-side objects (closest 0.18 m, front-right); no stop.
  - ⓘ Detours passed close: ~0.2–0.3 m from the box and the wall-side objects (Theta\* + inflation 0.8 / scaling 6: the cost band is zero beyond 0.8 m from an obstacle, so the path centre sits there and the hull 0.22 m off).
  - ✅ **Clearance (operator OK): GLOBAL costmap inflation 0.8 / 6 → 1.1 / 4** (planner only; the local costmap the controller and the 09-17 lane view use stays 0.8 / 6). Set live by `ros2 param set`, then run `detour_clear1_125744` (low box near face 6.07 m, slightly right; memory on): **SUCCEEDED**, left detour with the centre 0.92–1.23 m left; **minimum hull-to-box gap 0.36 m** (odometry + box geometry, heading included) vs **0.24 m** on the previous run; nearest anything on the safety scan **1.18 m** (was 0.18 m to the wall-side objects). The closest moment is the front corner passing the box's corner while still angling out (+14.7°). Slow zone only in the final approach (angled at the wall-side objects). Written into `nav2_dagger.yaml`.
- **Defaults (operator OK, 2026-10-03): `near_field:=true` and `low_obstacle_memory:=true`** in nav2.launch.py and nav2_gps.launch.py — driven all day with no false stop; `:=false` reverts either.

**Q69 — Foxglove click-to-goal: the goal slid while the rover drove to it (2026-10-03)** ✅ *fixed by using `odom` as the display frame*
- *Operator:* "it avoided the obstacle but got confused about the goal … I gave a point straight behind it but it changed the goal as it moves" (not recorded).
- *Cause (measured live, parked):* GNSS 3D fix only, no RTK, h_acc **2.8–3.5 m**; **map→odom moved 0.68 m in 27 s** (yaw steady) — the GPS position walk fed through `gps_map_odom`. Claude had recommended display frame `map` (the documented setup, Q-section above); a goal clicked in `map` is pinned to that wandering frame, and Nav2 re-reads it in `odom` at every replan → the goal slides 1–2 m over a minute's drive.
- *Fix:* clicked goals in **`odom`** (display + fixed frame) — the rover's continuous frame (FCU velocity + gyro + GNSS heading, no GPS position), the one the planner, costmaps and low-obstacle memory use. `map` stays for lat/lon goals and SLAM. `goal_pose_relay` now says "set the display frame to odom" on a refused goal and WARNs on a `map` goal in GPS mode; COMMANDS.md updated.
- ⓘ Lat/lon goals inherit the same ±2–3 m without RTK (P6).

**Q70 — Foxglove goal session: goals held, but in-place turns near obstacles DEADLOCKED (2026-10-03, bag `detour_fox1_132302`, 254 s)** ✅ *fixed, live-verified (bag `detour_fox2_140521`)*
- *Session:* 3 goals clicked in `odom` (Q69). **No goal drift** — every plan ended within 0.03 m of its goal while map→odom wandered 2.55 m (max 3.0 m; GNSS 3D only, h_acc 2.1–4.1 m, heading 96 % valid). Goal 1 (9.1 m ahead) **SUCCEEDED** in 33 s, 0.43 m / 3°. Goal 2 (8.1 m behind) — 180° turn-around in ~6 s (0.25–0.49 rad/s actual), drove to **0.26 m** of the goal, then **stuck on the final heading turn** (125° to go) → "Failed to make progress" ×5 at 130 s; preempted by goal 3 (1.7 m behind-left), which never moved. Low-obstacle memory: 578 cells remembered (wall, plants), **0 near the 18.5 m driven** (replay).
- *Cause 1 — the slow zone scales TURNS:* at goal 2 the rover faced something 1.6–1.9 m ahead (28–104 rays in PolygonSlow); Humble's slowdown scales the whole twist → 0.45 → **0.18 rad/s** commanded, 0.06–0.12 actual (the operator: "rotation speed should be more"). Same Humble limit Q57 hit on the follow path.
- *Cause 2 — rotate-to-heading DEADLOCK:* the controller limits each new turn command to the MEASURED turn rate + `max_angular_accel` × 0.1 s = **0.12 rad/s** from standstill; the slow zone made it **0.05**, below the tracks' turn deadband (0.15 commanded → 0.03–0.08 actual, Q65) → no motion → measured rate stays 0 → command stuck at 0.12 → progress checker (no translation for 25 s) aborts. Goal 3 the same, plus STOP from 4–5 stop-zone rays.
- *Proposed (needs the operator's OK — the slow zone is on the stop path):* (1) a **forward-only slow zone** for Nav2: a small node between `/cmd_vel_nav_raw` and the collision monitor applies the same zone (x 0.85–2.6, ±0.85, > 3 rays, ×0.4, stale scan = slowed) to linear velocity only; PolygonSlow leaves the Nav2 monitor (stop zone + approach stay) — as follow_controller did in Q57. (2) `FollowPath.max_angular_accel` 1.2 → 2.5: a turn from standstill starts at 0.25 rad/s, above the deadband. (3) `rotate_to_heading_angular_vel` 0.45 → 0.6 (operator request), then re-check the turn-into-box stop margin (13 cm at 0.45, Q65).
- **Built (operator OK):** (1) **`nav_slow_zone`** (dagger_bringup): `/cmd_vel_nav_raw` → `/cmd_vel_nav_slow` → collision_monitor (input remapped; PolygonSlow removed from its list, kept in the yaml for reference; stop zone + footprint approach unchanged). Same zone (x 0.85–2.6, ±0.85, > 3 rays), ×0.4 on the WHOLE twist while the rover drives (an arc keeps its curvature — scaling only linear would tighten it 2.5×), **pure in-place turns (|v| < 0.02 m/s) untouched**, stale scan = occupied. (2) `FollowPath.max_angular_accel` **1.2 → 3.5**, not the 2.5 first proposed: the config history puts the tracks' breakaway at ~0.3 rad/s, and RPP's first command from standstill is accel × 0.1 s → 0.35. (Humble 1.1.20's RotationShim does support `closed_loop: false` / `rotate_to_goal_heading`, already set; the lock was RPP's rotate-to-heading, which always ramps from the measured rate.) (3) `rotate_to_heading_angular_vel` **0.45 → 0.6**. `tools/test_nav_slow_zone.py` **11/11** (the 0.12 rad/s turn start passes as 0.12; 3 rays not slowed, 4 slowed; beside/beyond the zone untouched; stale scan slows driving, not turns). `follow_gate_test --path nav` now exercises nav_slow_zone + the monitor.
- ✅ *Live, disarmed bench (T2 restarted; wiring verified: monitor input `/cmd_vel_nav_slow`, polygons Stop + FootprintApproach, FollowPath 0.6 / 3.5):* tall box on the centre line, near face 0.90 m from the bumper (33 rays in the slow zone, 0 in the stop zone). `follow_gate_test --path nav --vx 0 --wz 0.6`: **0.60 passed through, 29/29 samples** (was ×0.4 → 0.24). `--vx 0.3`: **0.12 = SLOW ×0.4, 29/29**. Stop zone and footprint approach published (4 vertices each).
- ✅ **Live Foxglove session 2 (`detour_fox2_140521`, 199 s, 4 goals in `odom`):** plans ended within 0.02–0.05 m of their goals while map→odom wandered 3.35 m. **Every in-place turn ran at the commanded 0.60 (monitor output = input):** turn-around **−148° in 4.5 s** (avg 0.61, peak 0.73 rad/s); the goal-2-type final turn facing obstacles **+155° in 5.7 s → SUCCEEDED, 2° off** (fox1: stuck at 0.05, aborted); a big final turn **+87° in 3.5 s → SUCCEEDED, 7°**. First turn commands 0.39–0.60 (≥ 0.35 from standstill). Goal 1 was preempted by goal 2 before its final turn; one transient "start or goal pose are an obstacle" (recovered). Goal 4 → **Q71** (canopy). Low-obstacle memory: 6 cells near the later path — 5 at a spot with a tall object within 0.5 m in ~190 clouds (likely the tall box left in the lane; avoided, not driven over), 1 single-cloud transient.
- ⓘ Not changed: the STOP zone still blocks in-place turns with something 0.5–1.45 m ahead (goal 3 also hit it); the footprint approach models turns exactly, so a forward-only stop zone is a possible follow-up.

**Q71 — ⚠️ Overhangs (canopies, branches, 0.75–1.6 m) are invisible to the PLANNER: a goal near the plant ended in a 36 s STOP and an abort (2026-10-03, `detour_fox2_140521`, goal 4)** 🟡 *built + tested offline (band top 1.35 m, overhang layer); live check pending*
- *What happened:* goal 4 lay at the end of the lane near the plant; approaching it (0.12–0.24 m/s, slow zone), the rover got to 0.45 m of the goal (tolerance 0.4) with 154° still to turn when **4–5 rays entered the stop zone's right-front corner** (x 1.42–1.45, y −0.64…−0.69): raw cloud shows the object **only at z 1.29–1.51 m** — the plant's canopy overhanging the path. The monitor correctly held STOP for 36.6 s while the controller kept asking for 0.12 m/s → progress checker abort. Same plant as Q66 run 1.
- *Cause:* the costmaps take returns 0.12–0.75 m high (/scan from Patchwork++, /scan_band) — a canopy at 1.3 m is not in them, so the planner (and the Foxglove costmap view) see open ground and route the rover under it; only the collision monitor's /scan_safety (0.30–1.60 m) sees it.
- *Proposed:* a separate **overhang layer** in both costmaps fed by `/scan_safety` (body-filtered, 0.30–1.60 m; its own marking + clearing, expected_update_rate 1.0) — the planner then keeps its 1.1 m clearance from canopies, branches and torsos, refuses goals inside them, and the operator sees them in Foxglove before clicking. Stop path unchanged.
- *Correction while building:* the layer alone would NOT have saved goal 4 — the canopy was 1.57 m from the goal centre, beyond the 1.1 m inflation, so the goal was legitimate for the planner; the STOP came from the stop zone seeing a canopy **above the rover**. In the zone the canopy sat at **1.41–1.47 m** (6,708 points, 162–199 s); the rover's tallest part is the LiDAR housing, **~1.17 m** (URDF cylinder 0.1 m around 1.12 m; operator confirmed nothing is mounted higher; the GNSS posts reach 1.11 m).
- **Built (operator OK: "it should not stop if something is taller than this"):**
  - **Safety band top 1.60 → 1.35 m** (`SAFETY_BAND_TOP` in nav2.launch.py, both sources of `/scan_safety`; safety_scan's default too) = rover top + 0.18 m for the 2° beam spacing (5 cm at 1.5 m) and ~4° of pitch. Everything that reads `/scan_safety` follows: both collision monitors, nav_slow_zone, follow controller, person/marker trackers (a person's torso stays in the band). low_scan's "tall" veto band stays 0.40–1.60 (not on the stop path).
  - *What 1.35 would have changed on every 10-03 drive (raw clouds through a model of the stop zone, > 3 rays):* exactly **two** stops disappear, both canopy entirely ≥ 1.38 m — fox2 goal 4 (369 clouds) and Q66 run 1 at the plant (334 clouds). Q66 checked separately because that goal sat 0.57 m from the plant: its lower canopy edge (1.24–1.35 m, 4–8 rays) and a branch (0.88–1.29 m, 9 rays) lay 0.05–0.7 m further on inside the zone's width, so the rover would have stopped ~0.2 m later, still clear. Slow-zone triggers lost: 4 short episodes (≤ 5 s), all ≥ 1.30 m.
  - **Overhang layer** (`overhang:=true`, default; `overhang:=false` reverts): a second `safety_scan` instance, **`overhang_scan`**, on the costmap's cloud (Patchwork++ non-ground with ground_seg, so a slope is not an overhang — unlike `/scan_safety`, which is raw) → `/scan_overhang`, band **0.75–1.35 m** (above `/scan` + `/scan_band`, up to the rover top), body box, range 0.5–30 m like `/scan`. New `overhang_layer` (ObstacleLayer, own grid so `/scan`'s empty rays under a canopy cannot erase it, normal clearing, expected_update_rate 1.0) in both costmaps, enabled by the overlay `params/overhang_layer.yaml`. Known limit: the LiDAR (+15° max) loses an overhang at h m within (h − 1.12)/0.27 m of itself (1.3 m → 0.7 m) and the mark clears; the collision monitor guards that last metre. **3.3 % of one core** at 10 Hz.
  - *Tests:* `tools/test_safety_scan.py` **33/33** (domain 78; 3 instances: rover = live /scan_safety 0.30–1.35, p2l reference pinned to the 1.60 band the captures were recorded with — still bit-identical, overhang = live /scan_overhang): bands, body frame invisible in the overhang band too, parked capture 0 returns < 1.6 m in both, walk-around 39–87 rays at all 8 spots; fox2 goal 4 — 1.60 reproduces the live STOP (100 % of 368 scans), 1.35 gives max 0 rays; Q66 plant's canopy edge / branch reach /scan_overhang (32–37 rays).
  - *Costmap replay (standalone global costmap with the real params, sim time, Q66 run 1 from 60 s, layer on vs off, domain 83):* layer loads and stays current; **the plant ahead is marked lethal from 74 s (11 → 55 cells at 1.4–4 m ahead)** — on the plan's line, where the old costmap had nothing; **0 lethal cells inside the footprint**, cost 0 at the rover centre throughout. Overhang-only cells: 51–92 % within 0.3 m of existing marks (the same plants/wall seen higher; mostly the plants beyond the steps on the left, 69–87 %), the isolated ones 9–20 m away beyond the right wall; nothing new within 6 m in the lane.

**Q72 — 🔴 Odometry STALLED while the rover drove: GNSS degraded near the lane end, odom said stopped/backing while it drove 1.1 m forward (2026-10-03, bag `detour_fox3_144415`, 178 s, 3 Foxglove goals)** 🟡 *open — fix to be agreed*
- *Session:* overhang layer + band top 1.35 live (T2 restarted, preflight PASS, `/scan_overhang` 9.9 Hz, both layers enabled, no warnings). **Goal 1 SUCCEEDED** (turn-around −153° and final +151° both at 0.60 rad/s, 10.5 m). **Goal 2** (10 m ahead, near the lane-end canopy): drove 9.7 m, then "start or goal pose are an obstacle" ×8 from 113 s → aborted 1.6 m short. **Goal 3** (1.1 m away, +151° turn): STOP for 26.7 s → "Failed to make progress".
- *Goal 2, cause — odometry, not the layer:* from 110.5 to 114.5 s Nav2 commanded **0.14–0.15 m/s forward**; the LiDAR range straight ahead fell **4.79 → 3.71 m** (static wall at −60° 2.93 → 2.07) = **~1.1 m driven at ~0.27 m/s**, while `/odometry/filtered` reported stopped then **−0.13…−0.22 m/s (net −0.28 m)**. Scan matching (ICP on `/scan`, 2 s windows) vs odom over the run: straight driving agrees within ±0.07 m per 2 s at GNSS h_acc 2.2–2.7 m / heading valid 100 %, but odom **under-reads by 0.32, 0.56, 0.52, 0.40 m per 2 s at 107–115 s, as h_acc grew 3.2 → 4.3 m and the dual-antenna heading went 100 → 0 % valid** (also 0.18–0.29 per 2 s at 45–51 s, heading 0–80 %). The odom twist comes from EK3 body velocity (VELXY = GPS), so degraded GNSS under the lane-end canopy / next to the wall corrupts it; the FCU's own speed loop uses the same estimate → the rover ran **faster than commanded** (0.2 m/s for 0.14, up to ~0.47 for 0.40). The canopy ahead (lower edge 1.24–1.31 m, a static object — nothing below it, so not a person) was therefore marked by the overhang layer at odom positions that crept 1.86 → 0.43 m toward the goal (0.55) as the rover "stood still" → goal cell lethal → planner abort. With true odometry it would have stayed ~1.3 m from the goal.
- *Goal 3, cause — stop zone vs in-place turn:* the goal was clicked from the (wrong) odom display; the canopy's lower edge **1.26–1.35 m** (inside the 1.17–1.35 safety margin) sat at the stop zone's far edge (x 1.40–1.45, y −0.4…−0.2); 4° into the turn **9–16 bins** were in the zone → STOP. The hull cannot reach x 1.40 by turning in place (corner sweep 1.03 m) — the stop zone blocking pure turns is the forward-only stop zone item (Q70).
- *Safety:* the collision monitor works in the rover frame from the live scan — it was unaffected (stops at 114.5 s on the command, held 0 for the turn). What is corrupted: costmap marks (smeared in odom), the plan, goal positions shown in Foxglove, and the FCU's speed control. preflight cannot catch a mid-run degradation; `fcu_odom_relay`'s max_plausible_speed only catches over-reads.

**Q73 — LiDAR odometry for P20: selected by research + benchmark on 6 drives; C++ node built, fixed, validated offline (2026-10-03/05)** 🟡 *offline 12/12 + 7 drives; live shadow run next*
- *Requirement (operator):* GNSS-independent odometry, **indoor and outdoor**. Constraints: no usable IMU (Cube caps RAW_IMU at 10 Hz, Helios-16P has none → FAST-LIO2 / Point-LIO / DLIO / LIO-SAM out unless an IMU is added); 16 beams; pivots at 0.6 rad/s; Orin CPU already busy. KISS-ICP was tried in July and removed (K2: jumps while pivoting) — its setup then was 1 m voxels (100 m default range / 100), all cores.
- *Research:* ISPRS GEOBENCH 2025 benchmark of 20 algorithms (indoor Bunker DVI, Livox MID360): LiDAR-only ATE I2EKF-LO 0.41 m, CT-ICP 0.81, GenZ-ICP 1.05, DLO 6.2, **KISS-ICP 23.5 (failed)**, LeGO-LOAM 35 (failed), GLIM 36 (failed). Potokar & Kaess 2025 (LO design ablations): plane-to-plane (GICP) > point-to-plane > point-to-point; constant-velocity deskew gives little without an IMU. Dropped: I2EKF-LO (ROS 1, Livox only), Kinematic-ICP (needs wheel odometry; ours is GNSS-based), CT-ICP/GLIM (heavy).
- *Benchmark* (`tools/lo_bench.py` + `tools/lo_eval.py`; same input for all: cloud in base_footprint, body box, 0.3-60 m, one thread; no ground truth → reference = direct GICP of two scans 5 s apart / of revisits / across each pivot, best of several starts, ≥ 60 % inliers): drives fox1-3, walkby (parked, people), tall2, slam_lane1 (520 s), slam_lane3 — 13 k scans. Ranges over the drives:

  | | KISS (July cfg) | KISS 1 m | GenZ-ICP | VGICP 1 m | **VGICP 0.5 m** |
  |---|---|---|---|---|---|
  | 5 s error p95 | 0.08-0.15 m | 0.08-0.15 | 0.06-0.12 | 0.07-0.13 | **0.07-0.09** |
  | 5 s yaw p95 | 0.5-1.7° | 0.5-1.7° | 0.2-1.2° | 0.06-0.26° | 0.06-0.42° |
  | pivots worst | 0.15 m / 1.2° | 0.16 / 1.0° | 0.18 / 1.8° | 0.08 / 0.1° | 0.13 / 1.0° (one 1° slip, fox1) |
  | revisits worst | 0.13 m | 0.13 | 0.18 | 0.33 (tilt drift) | **0.16** |
  | parked + people | 9 mm | 9 mm | 11 mm | 4 mm | 4 mm |
  | CPU ms mean / p95 | 45-63 / 94-119 | 42-61 / 87-118 | 158-244 / **1076-1273** | 26-30 / 34-36 | **32-40 / 42-52** |

  All held through fox3 107-115 s, where `/odometry/filtered` lost ~1.4 m. VGICP 1 m drifted in tilt on lane3 (pitch 12.9°, z −0.41 m → 0.2-0.33 m loop error); 0.5 m voxels fix it (pitch 4.0°). Gravity alignment from the 10 Hz `/mavros/imu/data_raw` (complementary filter) made it WORSE (pitch 17.9°) — dropped. **Selected: small_gicp VGICP scan-to-model, 0.5 m Gaussian voxels (LRU 100 / 10), 0.2 m downsampling, 10-neighbour covariances, constant-velocity guess, no deskew** — ~35 % of one core, finer voxels also suit indoors.
- **Built:** C++ `lidar_odom` (dagger_perception, vendored small_gicp 1.0.1; new `dagger_msgs/LidarOdomStatus`): body box + range as safety_scan, mount from /tf_static, `lidar_odom/odometry` (twist covariance from the 6×6 information matrix, floored; the weak direction of the translational information (Schur complement) inflated when < degenerate_info), rejects (too few points / inliers, jump) publish nothing, map reset after 10 rejects; `tools/test_lidar_odom.py` (domain 84). **Status:** synthetic 6/8 (room static, 0.3 m/s + 0.2 rad/s reproduced exactly, rejects, body box; corridor weak axis = x correctly but info 6.7 k > threshold 1 k — needs calibration: real scans median 150-180 k); **fox3 replay FAILS:** matches the Python pipeline scan for scan for ~35 parked scans, then the guess creeps (−0.003 → −0.014 → −0.02 m/scan) and it diverges; Python (float or double transform) stays < 1 mm. Not yet found (checked: Eigen, covariance setter, optimizer state, compile flags). Not launched anywhere — no effect on the rover.
- ✅ **Divergence ROOT CAUSE found + fixed (2026-10-05): rotation drift through `Isometry3d::inverse()`.** Isolation: a standalone C++ copy of the loop on the first 60 fox3 scans diverged too (to NaN) — so not the node; not threads (OMP 1 vs 6), not `-march`/FMA (`-ffp-contract=off`), not Eigen (3.3.9 / 3.4.0 / 3.4.1 bit-identical, also with vectorisation off), and the vendored small_gicp equals upstream v1.0.1. Registration against a fixed map, and map building at identity poses, matched Python to 1e-12. With the map FROZEN the loop still diverged → the motion-model arithmetic: small_gicp composes `T * exp(δ)` every iteration so R slowly stops being orthonormal; `Isometry3d::inverse()` uses Rᵀ, and through the constant-velocity guess `T * Δ` the error roughly doubled per scan (clean to ~25, visible ~30, NaN ~45). Python used a full matrix inverse. **Fix:** re-orthonormalise every pose and step (quaternion round-trip) — the C++ output then equals the Python reference digit for digit. Also: a gap > `max_gap` 2 s (or time going back) rebuilds the map (pose kept); `degenerate_info` 1 k → 15 k (synthetic 60 m corridor 6.7 k, real fox3 scans 35-180 k).
  - *fox3 lockstep replay (`tools/test_lidar_odom.py`):* 1767/1777 accepted — the 10 rejects/2 resets were the test feeding the bag into the node that still held the synthetic room (now handled by max_gap); CPU 38.5 / 52 ms mean / p95; **106.5-116 s (where `/odometry/filtered` read ~0.6 m): 2.85 m vs direct registration 2.82 m.**
- ✅ **Node validated offline on all 7 drives** (`tools/lo_node_bench.py`: the node in lockstep beside the Python reference, scored by `lo_eval.py`): **identical to the reference metric for metric (±1 mm)** — 5 s error p95 0.002-0.088 m, revisits worst 0.002-0.195 m, pivots 0.037-0.126 m (the known 1° slip on fox1); accepted 99.94-100 % (≤ 1 scan per drive), 1 degenerate scan in 15 k (translational information p1 18.6 k-114 k vs threshold 15 k); CPU 35-51 ms with three replays sharing the CPU, 38 / 52 ms mean / p95 alone. `tools/test_lidar_odom.py` **12/12** (synthetic incl. the corridor after a 3 s gap: 14/14 degenerate, weak axis x, var x 1.02 vs y 0.0006; fox3: 1777/1777 accepted, 0 resets, 106.5-116 s 2.85 vs 2.82 m).
- **Launch (built, default off):** T2 `lidar_odom:=false|shadow|true` (nav2_gps → localization.launch.py). `shadow` runs the node (`/lidar_odom/odometry`, `/lidar_odom/status`) without fusing — for the live comparison; `true` also fuses its **linear velocity** into ekf_local through `params/ekf_fused_lidar.yaml` (odom1: vx, vy; yaw stays gyro + dual-antenna heading; LiDAR floor 0.02 m/s vs FCU 0.1 m/s ≈ 25:1; degenerate axis 1 m/s → FCU takes over; rejected scan → nothing published → FCU alone). Checked by evaluating the launch description in all three modes.
- *Next:* live `lidar_odom:=shadow` during a normal drive (compare with `/odometry/filtered`, CPU by PID) → `true` → guard (stop path, needs OK).
- *Indoor note:* indoors the FCU has no position/velocity/yaw source (EK3 = GPS only) → GUIDED will need LiDAR odometry as ExternalNav (EK3 source set 2, operator's parameters) — later phase.

**Q74 — LiDAR odometry live in shadow: outdoor lane + indoor drive (2026-10-05, bags `detour_shadow1_112443` 154 s, `detour_shadow2_indoor_112857` 164 s)** 🟡 *LiDAR far better than /odometry/filtered; indoor corridor scale open*
- *Setup:* T2 `lidar_odom:=shadow` (EKF `odom1` not set → /odometry/filtered unchanged), operator driving MANUAL on the RC; indoor run started outside the door (GNSS degraded, not lost: 3D fix, h_acc 4-12 m, 16-25 sats, heading 0 %). FCU: `FS_EKF_ACTION 0` (EKF failsafe disabled) — noted, operator's parameter.
- *Live health (both):* 100 % of scans accepted, 0 rejected, 0 map resets, 0 degenerate, 0 dropped scans (dt > 0.15 s); CPU 58-60 / 75 ms mean / p95 per scan with the whole stack running (~44 % of a core); translational information ≥ 18.6 k outdoor, ≥ 21.7 k indoor.
- *Accuracy* (`tools/lo_bag_to_bench.py` → `lo_eval.py`, live outputs vs direct registrations; start→end = last scan registered against the first):

  | | outdoor LiDAR | outdoor /odometry/filtered | indoor LiDAR | indoor /odometry/filtered |
  |---|---|---|---|---|
  | 5 s error median / p95 / max | **0.03 / 0.09 / 0.13 m** | 0.21 / 1.02 / 1.33 | 0.07 / 0.28 / 0.44 | 0.55 / 2.0 / 2.5 |
  | revisits | **0.27 m** | 3.9-5.2 | **0.27-0.33** | 1.4-2.4 |
  | turn ~100° moving ~1 m | **0.02 m** | 0.97 | **0.05** | 1.05 |
  | start→end | **0.20 m / 1.1°** | 0.61 / 2.0° | **0.26 m / 0.0°** | 1.26 m |

- *Where /odometry/filtered goes wrong (per 2 s, yaw rate agrees in every window — only linear velocity is off):* outdoor with good GNSS (h_acc 2.1-2.6 m, heading 100 %) LiDAR and FCU speed agree within ±0.04 m/s; as h_acc > 3 m / heading 0 % the FCU reads 0.1-0.3 m/s LOW, and in every turn (110-118 s: LiDAR 0.25, FCU ~0); indoor up to 0.52 m/s low (0.15 vs 0.55). 30 outdoor moving windows off by > 0.08 m/s.
- ⚠️ *Indoor corridor stretch (64-84 s, LiDAR's weak axis = forward, information 27-39 k):* LiDAR forward distance is 6-13 % ABOVE a direct registration of the ground-free clouds (`/patchworkpp/nonground`), 4-19 % above the all-points one; elsewhere 0-3 %. Not the motion model or convergence: tighter convergence (0.1 mm, 50 iterations), half or no constant-velocity translation all give identical results. The references themselves are biased toward zero motion by the 16-beam rings (the ground-free one moved toward the LiDAR by up to 0.2 m; ceiling rings remain) → **which is right needs a taped distance.** Note: raising degenerate_info to flag these stretches would hand the forward axis to the FCU velocity, which indoors is far worse — keep 15 k.
- *Next:* (1) indoor tape test — a measured straight run along the corridor between two marks, both directions; (2) outdoor `lidar_odom:=true` with Nav2 goals.

**Q75 — LiDAR odometry FUSED live: Foxglove goal session (2026-10-05, T2 `lidar_odom:=true`, bag `detour_fused1_120512`, 128 s)** ✅ *fusion works; residual = heading*
- *Pre-drive check:* `ekf_local` has `odom1 /lidar_odom/odometry` (vx, vy) beside `odom0 /fcu/odom`; no warnings; parked LiDAR vx +0.0001, fused −0.0001. Live LiDAR twist σ ≈ 0.033 m/s (floor 0.02 + registration) vs FCU 0.1 → **~9:1** (not the 25:1 estimated from the floor alone).
- *Session:* 3 goals in odom (5.2 m, 13.1 m, 6.3 m driven) — **all SUCCEEDED**, closest 0.30-0.33 m, final yaw 0-10° off; in-place turns 0.55-0.63 rad/s (commanded 0.60, monitor passed them unchanged); collision monitor never intervened; no warnings. GNSS h_acc 1.9-3.2 m, heading valid 82 % (better than fox3). lidar_odom: 100 % accepted, 0 resets, 1 gap > 0.15 s, CPU 59 / 77 ms.
- *Fused speed tracks the LiDAR in every 2 s window* (0 windows > 0.08 m/s apart, most ±0.02); the 0.04-0.06 gaps are decelerations into turns where the FCU speed lags. Per goal the fused odometry moved 3-5 % less than the LiDAR (residual FCU pull).
- *Accuracy vs direct registrations:* 5 s position p95 / max LiDAR 0.09 / 0.18 m, **fused 0.11 / 0.14 m** (old odometry in shadow1: 1.02 / 1.33); start→end LiDAR 0.06 m, fused 0.24 m / 1.1° (old: 0.61 m / 2.0°). **Remaining fused error is HEADING:** 5 s yaw p95 fused 1.3° vs LiDAR 0.4°; revisits fused 2.0-2.5° vs LiDAR 0.04° — the fused yaw comes from gyro + dual-antenna heading (noisy, intermittent), which costs the extra ~0.1 m.
- ✅ **`lidar_odom:=true` is the T2 default (operator OK, 2026-10-05)**; the node now only starts in `odom_source:=fused` (the only mode that fuses it). `lidar_odom:=false` reverts.
- *Recorder:* `detour_record.sh` now also records the EKF's other inputs (`/fcu/odom`, `/mavros/imu/data_raw`, `/gps/heading_imu`) so ekf_local can be replayed offline with variants (e.g. fusing the LiDAR yaw rate, less weight on the GNSS heading).

**Q76 — Final 180° turn HUNTED at the goal: tracks pivot behind base_footprint + a fixed 0.4 m goal tolerance (2026-10-05, bag `detour_heading1_122047`, goal 4)** ✅ *fixed, live-verified (bag `detour_sticky1_124552`)*
- *Symptom (operator: "at the last 180° goal it was not able to get its goal position"):* goals 1-3 SUCCEEDED; goal 4 arrived 0.36 m out with ~165° to turn, then from 184 s ~40 short alternating turns (−20…−50° / +5…+30°) and 5-10 cm creeps for ~2 min (net −400°), "RPP detected collision ahead", "Controller patience exceeded", "Failed to make progress".
- *Mechanism:* RPP rotates to the goal heading only while the goal is within the goal checker's xy tolerance (it reads `getTolerances()` every cycle — "Unable to retrieve goal checker's tolerances!" in the library). The turn moved base_footprint 0.36 → 0.42-0.44 m; outside 0.4 m RPP drives to the goal again, which is now behind it at more than `rotate_to_heading_min_angle` → turns back toward it, creeps in, turns to the heading, drifts out... The old FCU-based odometry read ~0 m/s in turns, hid the drift, and the same 150-160° final turns succeeded on 10-03 (Q70).
- *Pivot point (from LiDAR odometry, translation vs rotation of each in-place turn):* clean 87-165° turns at 0.6 rad/s (fox1-3 replays): **0.07 m behind base_footprint** (IQR −0.07…−0.06, y −0.02); short stop-start turns during the hunting: 0.17-0.20 m. A clean 180° turn moves base_footprint ~0.14-0.25 m, a jerky one up to 0.38 m. Not worth moving base_footprint in the URDF.
- **Fix (operator OK): `dagger_nav_plugins::StickyGoalChecker`** (new package `dagger_nav_plugins`, replaces `nav2_controller::SimpleGoalChecker` in `nav2_dagger.yaml`): identical, except once the goal position is reached (0.4 m) the xy tolerance it reports — and checks — widens to `xy_goal_tolerance_sticky` **0.8 m** until the heading is reached (0.2 rad) or the goal changes; drifting beyond 0.8 m re-arms the normal approach. `reset()` is ignored (the controller server may call it on every replan of the same goal); a new goal is recognised from the goal pose (moved > 0.25 m or turned > 0.1 rad). Stop path untouched.
- *Tests:* gtest 4/4 (`colcon test --packages-select dagger_nav_plugins`): the live case (arrive 0.36 m / 163°, replan + reset, drift 0.44 m mid-turn → still turning; heading reached 0.5 m out → SUCCESS), new goal clears the state (incl. same position / new heading), drift > 0.8 m re-arms, ordinary goals as SimpleGoalChecker. Loaded by a real `controller_server` (isolated domain): "Created goal checker : general_goal_checker of type dagger_nav_plugins::StickyGoalChecker".
- ✅ **Live (T2 restarted with the new yaml + LiDAR fusion default; bag `detour_sticky1_124552`, 5 goals, operator: "worked so good"):** every goal logged "goal position reached (0.39-0.40 m) -- xy tolerance 0.80 m", made ONE final turn (−169°, +155°, −109°, −168°, +150°; 0.42-0.60 rad/s) and "Reached the goal!" 4-10 s later — no hunting. base_footprint moved 0.06-0.17 m (fused) / 0.07-0.27 m (LiDAR) during those turns; success 0.39-0.61 m from the goal point, final heading ±8-11° (tolerance 11.5°). Collision monitor never stopped (slowed 18 % of samples). One transient "start or goal pose are an obstacle" mid-turn (132.5 s), goal still succeeded. *Known cost:* after a big final turn the rover sits up to ~0.6 m from the clicked point — the pivot geometry, now visible rather than hidden.

**Q77 — Odometry heading tuning by offline EKF replay; FCU overspeed measured; LiDAR twist spikes at turn ends (2026-10-05, bag `detour_heading1_122047` 323 s + today's GUIDED drives)** ✅ *keep the live EKF set-up; no stop-path guard needed on this data*
- *Tooling:* `tools/ekf_replay.py` reruns `ekf_local` (installed yaml + overlays + per-variant overrides) on sim time from a bag that holds the EKF inputs (recorder since Q75), one isolated domain per variant, 2× speed; `tools/ekf_replay_score.py` merges the outputs into a `lo_bag_to_bench.py` file for `lo_eval.py`. Sanity: V0 (the live set-up replayed) vs the recorded `/odometry/filtered`, relative motion: 0.11 m median / 0.20 max over 47 m, yaw 0.15° median / 1.0° p95 (replay timing).
- *Results (revisits, 90 pairs up to 300 s apart, p95 position / heading):* LiDAR odometry alone **0.11 m / 0.30°**; live `/odometry/filtered` 0.64 m / 1.5°; V1 + LiDAR yaw rate 0.70 / 1.6; V2 GNSS heading σ 0.5° → 3° 0.75 / 2.7; V3 no GNSS heading (gyro + LiDAR yaw rate) 0.60 / 4.6; V4 gyro only 0.76 / 7.3; V5 LiDAR POSE fused (x, y, yaw, odom1_relative, no GNSS heading) 1.38 / 7.2; V6 V5 + GNSS heading 1.41 / 2.0. 5 s errors: all EKF variants 0.105-0.11 m p95 (pose variants 0.28), LiDAR 0.085. Heading vs GNSS heading over the drive: LiDAR −0.6° (σ 0.47), V3 −3.8°, V4 +6.7° (the gyro outweighs the LiDAR yaw rate and its bias wanders). **→ Keep the live set-up** (gyro + GNSS heading σ 0.5° + FCU and LiDAR vx, vy). The remaining 5× gap to the LiDAR pose is inherent to integrating velocity along a ±1-2° heading; closing it means making lidar_odom the primary odom (EKF as fallback) — noted as a future option, not justified now.
- *FCU overspeed (actual LiDAR speed − commanded, 1 s windows, 4 GUIDED drives incl. the degraded fox3):* median −0.03…+0.01, p95 +0.05…+0.10, max +0.08…+0.16 m/s; the one +0.30 window (fused1 92 s) was an in-place turn's end with a LiDAR twist spike, not overspeed. Well inside the stop-zone margins (stop from 0.6 m/s = 0.29 m) → **a stop-path speed guard is not justified on this data**; revisit if a drive shows more.
- ⚠️ *LiDAR twist spikes:* 1-3 % of scans outdoors jump > 0.4 m/s between scans (> 4 m/s², e.g. +1.64 m/s at fused1 93.0 s between −0.52 and +0.22), 90-100 % of them during or within 0.5 s of a turn — registration snapping when a 0.6 rad/s rotation stops (no deskew: a scan spans 3.4° of turn); indoors 69 spikes, mostly not turn-related. The node's jump check (2 m/s) passes them; the EKF smooths them (fused `/odometry/filtered` |vx| > 0.8 m/s in 0-5 samples per drive; the old FCU-only odometry had 12 in shadow1). Candidates: an acceleration gate in lidar_odom (inflate the twist covariance when |Δv| > a_max·dt), deskew with the constant-velocity model.

**Q78 — LiDAR twist spikes: acceleration gate tried and REVERTED (2026-10-05)** ✅ *no change; keep lidar_odom as validated*
- A gate (inflate the twist covariance when |Δv| > 3 m/s²·dt + 0.1) halved the fused spikes in the `heading1` replay (> 0.8 m/s: 85 → 36) but made accuracy WORSE (5 s error p95 0.110 → 0.163 m, pivot 0.033 → 0.063 m): the "spikes" are partly real displacement the registration under-reports during a fast turn and catches up when it stops — gating throws it away. A twist from a 0.3-0.5 s pose window would spread it instead (spikes 100 → 1-7 in replay) but was not pursued: the live effect is small (fused odometry 0-5 samples > 0.8 m/s per drive) — operator: don't over-engineer. Revisit only if a drive shows a real problem.

**Q79 — Indoor tape test: LiDAR odometry within 1-2 % of ~7 m; the FUSED odometry 21-27 % short indoors — FCU velocity weight (2026-10-05, bag `detour_tape1_135431`)** ✅ *fixed (operator OK): FCU velocity σ 0.3 with LiDAR fusion; live check at the next drive*
- *Run (operator, MANUAL on the RC, corridor):* marks A, B ≈ 7 m apart (operator: "approx 7 m"), front of the rover as reference; A (10 s still) → B (10 s) → rotate in place, re-align the front on B (10 s) → A (10 s). GNSS h_acc 2.2-17.3 m, 11-28 sats.
- *Front-point travel between the still moments:* **LiDAR odometry 6.96 m (A→B) and 6.85 m (B→A)** — within ~0.6-2 % of the tape; **no 6-13 % corridor over-read** (that figure in Q74 came from the ring-biased direct-registration reference). Fused `/odometry/filtered`: **5.52 / 5.11 m (−21 / −27 %)**.
- *Cause — weighting, not degeneracy:* lidar_odom not degenerate (information 26-34 k), but its twist σ indoors is 0.057 m/s (outdoors 0.033) while the FCU velocity keeps σ 0.1 → only ~3:1; the FCU velocity indoors reads 0.16-0.24 m/s against the LiDAR's 0.29-0.35 and drags the fused speed to 0.22-0.25.
- *Replay (`tools/ekf_replay.py` F-variants scale the FCU twist variance by a relay; V0 reproduces live 5.51 / 5.10 m):* FCU σ 0.2 → indoor 6.44 / 6.23 m, σ **0.3 → 6.71 / 6.56 m (−4 / −6 %)**; outdoor (heading1) 5 s distance vs the LiDAR p95 0.068 → 0.047 / **0.043 m**. Spike counts in the fused output vary between replays (79-147), no consistent cost.
- ✅ **Applied:** with `lidar_odom:=true` (default), `fcu_odom_relay` `twist_variance` 0.01 → 0.09 (σ 0.3 m/s) in localization.launch.py; verified on a running relay (true → 0.09, false → 0.01) — the LiDAR then outweighs the FCU ~9:1 indoors and ~80:1 outdoors; when a scan is rejected or a direction is degenerate (+1 m/s σ) the FCU velocity still carries it.

## Next session plan — obstacle avoidance day 2 (written 2026-10-02 wrap-up)
Gap 1 (near field, P13) done today (Q65). Tomorrow, gap 2 (P14), hand-off prompt `docs/reports/next_session_prompt.md`:
1. **Approach-and-stop baseline in the test lane** (no detour: the lane is a wall on the right and steps on the left, which Nav2 cannot see — P18): low cardboard box 20–25 cm × 40–60 cm, 8 m ahead at the rover's distance from the wall (~2.2 m); `bash tools/detour_record.sh approach_low`; operator sends a 6 m goal parallel to the wall; the rover stops ~2 m before the box. Shows when the box is erased from both costmaps and gives real clouds for replay. T2 with `near_field:=true`.
2. **Low-obstacle memory v2** (user OK'd the outline): C++ `low_scan` in dagger_perception on `/patchworkpp/nonground` — low-only scan in one pass (tall check ±2 bearings) + persistence gate (~0.5 s in the same ~15 cm odom cell; people move, boxes don't) → `/scan_low` → existing `low_obstacle_layer` (clears only beyond 4 m); replaces p2l_low_all + p2l_tall + low_scan_filter (two p2l would cost ~13 % of a core each). Behind `low_obstacle_memory:=true`.
3. Offline: synthetic; replay `body_walk_132959` (operator circling → no mark); replay the approach bag (box kept to the stop). Live: walk-by at 2–5 m (no blob), approach again with memory on.
4. Later: detour test (gap 3) at a site with room or the step side blocked by tall markers; flip `near_field` default after a clean drive; P18 drop-off detection; avoidance on the follow path.

## Next session plan — obstacle avoidance (written 2026-10-02) — superseded by "day 2" above
User chose to close the obstacle-avoidance gaps next, in a new session. Hand-off prompt: `/home/jetson/dagger_reports/next_session_prompt.md`. Order: (1) near-field blind zone of `/scan_safety` (P13) — raw-cloud capture parked, self-filter + range_min ~0.3, needs the user's OK; (2) low obstacles forgotten inside ~3.3 m (P14) — revisit the shelved low-obstacle memory with the Q32 ideas; (3) a dedicated live detour test (tall box, then low box, placement from the beam geometry); (4) later: avoidance on the follow path. Status estimate at hand-off: object following ~80 %, SLAM ~70 %, waypoint following ~70 %, obstacle avoidance ~55 %, road route 0 %.

## Next session plan — following (written 2026-09-30 wrap-up)
**State:** person click-to-follow and ArUco-marker following both DRIVEN live (reid3, aruco2). GitHub in sync at `652cd4e`. Command reference: `ros2_ws/COMMANDS.md`.
**Bring-up:** T1 `drive_bringup use_camera:=true` → T2 `nav2_gps` → preflight PASS → T4 `follow.launch.py det_conf:=0.2 marker_dictionary:=DICT_6X6_50 marker_ids:=0 marker_size:=0.175` → `bash tools/track_record.sh <name>` → page `http://<rover-ip>:8080`.
1. **Re-test CLEAR live** (Q61 fix is offline-tested only): follow the marker, press CLEAR → the rover must stop and NOT turn; state `disabled: selection cleared`.
2. **LiDAR hold of the marker's carrier** (Q61): marker hidden with the carrier still in front → today 3 s hold then a full sweep. Design: hold the connected LiDAR return at the marker's last position, continuity-gated, target source `lidar`; resume on the marker.
3. Marker on a white board with ≥ 3 cm margin; then check the second detection pass live in sunlight (Q60 — proven on recorded frames only).
4. Parked by the user (09-30), in this order when asked: near-field blind zone of `/scan_safety` (P13, needs OK) · crowd test with 2–3 helpers + end-to-end video latency (P12) · Nav2 low-obstacle detour (P14) · FCU items (P4, P5, speed overshoot P16) · GPS waypoints in open sky (P6).

## Next session plan — SLAM (written 2026-09-26 wrap-up)
**State:** SLAM step 3 built (b468da5 → 693b39d). Map `lane3fix` (+`_geo.yaml`). Valid live evidence so far = the PARKED test only (17:49). The loc1 drive is invalid (Q44/Q45). T5 on the rover still runs the broken build → **restart T5 before anything; no Nav2 goals until then.**
1. ✅ (Q46) **Score the loc1 replays with the map loaded** (left running at wrap-up): `python3 tools/slam_replay_eval.py ~/dagger_bags/slam_loc1_180107 --loc m_base m_odom` (+ `grep LOADED ~/dagger_bags/replay/m_*/geo.log`). Decide: is along-lane sliding still there with the map anchoring it? Only if yes, adopt `distance_variance_penalty 0.2 / minimum_distance_penalty 0.2` (m_odom) for localization.
2. ✅ (Q47) **Live, parked:** T2 `map_source:=slam`, T5 `mode:=localization map:=lane3fix` → expect `map LOADED … 100 %`, `seeded SLAM`, `localized`; preflight PASS.
3. ✅ (Q47) **loc2 RC drive** with `bash tools/slam_record.sh loc2` (+ measure slam_toolbox CPU while driving, by PID from a script file — H19).
4. ✅ (Q48) **First Nav2 goal in the SLAM map** (user sends it; goal tolerance 0.4 m vs map->odom jitter).
5. ✅ (Q49/Q50) Step 4: lat/lon goals → SLAM map via `_geo.yaml` (carries the session's 2–5 m GNSS bias; RTK would remove it).
**Bring-up for SLAM localization:** T1 `drive_bringup` → T2 `nav2_gps.launch.py map_source:=slam` → T5 `slam.launch.py mode:=localization map:=lane3fix` → preflight. Back to GPS: T2 without `map_source`, T5 without `mode`.

## H. Diagnostic pitfalls (continued)

**H12 — Stale ros2 daemon shows 0 publishers for live topics** — `ros2 daemon stop; ros2 daemon start`.

**H13 — Killing a `ros2 launch` parent leaves orphan nodes** (joint_state_publisher, static TF, system_ready) that duplicate the next launch → check `pgrep -fa` after stopping a terminal.

**H14 — Bag unreadable while recorder still running** — stop `pf_record.sh` before analysis.

**H15 — Route check did not model "turn to segment heading, then drive straight"** (user caught it) — `tools/route_check.py` scores segments only; also fan the headings (0/9/18/27/35/45°) and check clearance along each.

**H16 — CTE measured against the latest replanned path hides route deviation** — compare against the ORIGINAL route (pf_analyze `--from-abs`).

**H17 — Auto-mode classifier blocks FCU param writes via mavros** — Claude gives the exact `ros2 param set /mavros/param …` command; the user runs it. Single `GetParameters` calls read reliably; looping `ros2 param get` times out.

**H18 — Documentation silently stopped (07-13 → 09-17)** — report folder unwritable after the user switch, and not flagged. *Fix:* folder group `dagger`, g+rwX, setgid (09-17).

---

## Current configuration (verified 2026-09-17)

**FCU (read live via mavros):** `SERVO1_FUNCTION=73`, `SERVO3_FUNCTION=74`, SERVO1/3 1100/1500/1900 not reversed · `ATC_STR_RAT_FF=0.8 P=0.3 I=0.2 D=0 IMAX=0.2 MAX=120` · `ATC_TURN_MAX_G=0.6` · `MOT_STR_THR_MIX=0.5` · `CRUISE_SPEED=0.4 CRUISE_THROTTLE=45` (was 2.0/50, changed 09-17, N8) · `WP_SPEED=2.0` · `EK3_SRC1_YAW=2` (GPS dual-antenna) `POSXY=3 VELXY=3` · `COMPASS_ENABLE=1` · `GPS1_TYPE=9` (DroneCAN C-RTK 2HP) `GPS1_POS_X/Y=0.33/0.29` · `MODE_CH=5`, `MODE6=15` (GUIDED) · `FS_THR_ENABLE=1 FS_ACTION=2` · `SYSID_MYGCS=255` · `SERIAL2_PROTOCOL=2` (RFD868).

**Nav2 (`nav2_dagger.yaml`):** Theta* · RotationShim (`angular_dist_threshold 1.2`, `forward_sampling_distance 2.0`, `closed_loop false`, `angular_disengage_threshold 0.3`, rotate 0.45 rad/s, accel 1.2) + RPP (v 0.4, lookahead 1.5 [1.2–3.0], approach 1.5 m / 0.10 m/s) · goal tol xy 0.4 / yaw 0.2 · footprint ±0.85 × ±0.58 · inflation 0.8 / scaling 6.0 · scan band 0.35–0.75 m, `inf_is_valid`.

**Odometry:** `fcu_odom_relay` (deadband 0.02, pivot gate 0.25, zero lateral) → `ekf_fused` (EK3 vx/vy + raw gyro + GNSS heading) → `odom→base_footprint`; `gps_map_odom` → `map→odom`.

**Bring-up:** T1 `drive_bringup.launch.py` → T2 `nav2_gps.launch.py` → (T3 `foxglove.launch.py`) → `python3 tools/preflight.py` **must PASS** → position + route/heading-fan check → `bash tools/pf_record.sh <name>` → user sends goal (`tools/send_goal.py` or Foxglove). E-stop: `tools/estop.sh` / `tools/estop_clear.sh`.

## Open items (updated 2026-10-02)
- **P12** 🟡 Click-to-follow (Q54–Q58): built, live-tested driven (reid2/reid3). Left: a deliberate **crowd / crossing test** with 2+ helpers (so far only incidental bystanders), and an **end-to-end (glass-to-glass) video latency measurement** — design estimate 80–130 ms; measured so far only in pieces (detections ~81 ms old at the tracker, ui_server +2–4 ms), plus the page's age / RTT meters.
- **P15** 🟡 **ArUco-marker following (Q59–Q61):** DRIVEN live (aruco2): follows, stops 3.1 m behind the marker, holds inside 1.6 m, recovers from every loss of sight. Left: **LiDAR hold of the carrier** (marker hidden → 333° sweep with the carrier still in front); re-test CLEAR live (fixed offline); remount the marker on a white board with ≥ 3 cm margin; second detection pass still unproven live.
- **P16** ⓘ Odometry slides 0.03–0.19 m backward while standing after turns / stops (Q61; LiDAR says no motion) — P10 family; FCU speed overshoot to 0.74 m/s on the 0 → 0.6 ramp (user's FCU tuning).
- **P17** 🟡 Handset video latency / CPU (user 10-02: "fine for now, keep for the future"): LITE at range (share of the MK32's ~45 ms decode/draw); D435 colour 60 fps (~15 ms less camera delay, more USB + CPU, untested); physical MK32 switch → RC channel → `/mavros/rc/in` for follow STOP / FOLLOW.
- **P13** ✅ **Near-field blind zone of `/scan_safety`** (fixed with T2 `near_field:=true`; default flip after a clean drive) (range_min 1.0 m; footprint reaches 0.85 m ahead, corners sweep 1.03 m) — both collision monitors. Follow-me mitigated in the controller (Q58); Nav2 in-place rotations and second persons are not. **Measured (Q65):** a person beside either side gives 0 rays today; the body box x −0.90…0.50, y ±0.60 + range_min 0.3 sees them (81–87 rays) with 0 false returns parked. **Built** (Q65): C++ `safety_scan` (dagger_perception), T2 `near_field:=true`, offline 25/25 and bit-identical to p2l; PolygonStop inner edge 0.85 → 0.50 in both monitors. Offline suites all pass at both range_min. **Live disarmed bench PASSED** (10 min no false trigger; turn cut to 0–×0.07 / STOP with the operator beside the hull; forward STOP held at the bumper and leaning). **Armed Nav2 turn into a box: stopped after 22° with 13 cm to spare, no coasting.** Left: drive with `near_field:=true` (gap 2/3 tests) and, if no false stops, make it the default. Known limits: > 3 rays needed (thin < 3 cm ignored), phantom edge returns (conservative), children / low objects beside the hull invisible.
- **P20** 🔴 **Odometry depends on GNSS velocity** (Q72): near walls/canopies (h_acc > 3 m, heading invalid) `/odometry/filtered` under-reads by up to 0.28 m/s and the FCU overspeeds; costmaps smear, goals go wrong. Needs a GNSS-independent motion source (LiDAR odometry) and/or a runtime odom-vs-scan guard. **Q73:** VGICP 0.5 m selected by benchmark; C++ `lidar_odom` fixed (rotation drift through Isometry inverse) and equal to the reference on 7 drives; T2 `lidar_odom:=shadow|true` built (default false). **Q74 live shadow:** LiDAR 0.20-0.26 m start→end vs 0.61-1.26 m for /odometry/filtered; indoor corridor scale (+6-13 %?) needs a tape test; next `lidar_odom:=true` outdoors.
- **P19** 🟡 **Overhangs (0.75–1.35 m: canopies, branches) were invisible to the planner** (Q71): built 2026-10-03 — overhang layer (`/scan_overhang`, both costmaps) + safety band top 1.60 → 1.35 m (things above the rover no longer stop it); offline 33/33 + costmap replay. Left: live check (canopy visible in Foxglove, goal 4 repeated).
- **P18** ⚠️ **Negative obstacles (drop-offs) are invisible to Nav2** (Q32, made concrete 2026-10-02): the test lane has steps (a drop-off) along its whole left side with plants beyond; the costmap shows 4–7 m of free space there. Claude, choosing a detour-test line from the costmap alone, first proposed a heading toward the steps — the operator caught it. Until a drop-off detector exists (D435 depth or a ground-continuity check on the LiDAR), keep autonomous goals and detours away from edges, block edges with tall markers in tests, and ask the operator about drop-offs before using the costmap to pick a line.
- **P14** ✅ *live-verified 2026-10-03 with `low_obstacle_memory:=true` (Q68): low box remembered to the bumper, detour held, no contact; walk-by leaves nothing; default still false.* Was: Nav2 low-obstacle detour (user's "task 2", deferred): ground segmentation sees low obstacles from 4–6 m but they are forgotten inside ~3.3 m (LiDAR blind band) before the planner commits.
- **P1** ⚠️ Always rerun full preflight after any mavros restart (BODY_NED resets).
- **P2** ✅ Ground segmentation — Patchwork++ feeds the costmap (642c846), floor 0.35 → 0.12 m; see P14 for the near-range limit.
- **P3** ✅ Collision monitor live-validated (N7); RC transmitter in hand as E-stop.
- **P4** 🟡 FCU "PreArm: Radio failsafe on" / `FS_THR_ENABLE=1` → user decision (parked).
- **P5** 🔴 N6 track imbalance → `SERVO1_MAX` calibration (parked).
- **P6** 🔴 GPS waypoints in OPEN sky vs Mission Planner AUTO; snap-to-free-space for GPS goals.
- **P7** ✅ Speed 0.6 m/s validated for follow-me (WP_SPEED); Nav2 still at its own cap — re-check final approach before raising it.
- **P8** 🟡 KISS-ICP differential x/y as a third odom input (parked).
- **P9** 🟡 L9 p2l wedge is worked around, not fixed.
- **P10** ✅ ZUPT on neutral track PWM in `fcu_odom_relay` (d0c42f0).
- **P11** ✅ Follow-me (Q0–Q53) — superseded by click-to-follow (P12).
- Goal tolerance (0.4 m vs map->odom jitter): user "fine for now".

## Parked items (user decision or later work) — reviewed 2026-09-25
| # | Item | Risk of leaving it | When it starts to hurt |
|---|---|---|---|
| P5 | Track imbalance → `SERVO1_MAX` 1830 | low | lane-level road following (0.1 m RMS drift eats the margin); higher speeds |
| P4 | `FS_THR_ENABLE=1` + dead-TX era failsafe | **medium** | any long mission: an RC glitch drops GUIDED → HOLD mid-run |
| — | `COMPASS_ENABLE=1` (yaw comes from GPS `EK3_SRC1_YAW=2`) | low–medium | unknown whether intentional; if the mag is bad it can trip pre-arm or fight EK3 — verify once |
| P10 | EK3 velocity lag after hard stops (~0.3 m false backward odom slide) | low | after every collision-monitor stop: costmap smear + a wrong restart pose |
| P9 | p2l lazy-subscribe wedge (watchdog + respawn only) | medium | both `/scan` and `/scan_safety` come from p2l; watchdog + freshness gate cover it, but the root bug is upstream |
| P8 | KISS-ICP as third odom input | low | GPS-denied stretches, long routes |

## Session plan — follow-me (updated 2026-09-25, end of build phase)

**Stages 1–5 are BUILT and bench-verified offline (Q1, Q4–Q10). Nothing has driven yet.**
Bring-up is now T1 `drive_bringup use_camera:=true` → T2 `nav2_gps` → T3 `foxglove` → T4 `follow.launch.py`.

**Remaining, in order — all of it on the rover:**
1. **Safety gate bench test (Q1), rover DISARMED.** `python3 tools/follow_check.py` (read-only, Claude can run), then **user runs** `python3 tools/follow_gate_test.py` and walks in: expect CLEAR → SLOW (×0.4) → STOP (0) → CLEAR, and `FootprintApproach` vertices ≥ 3 (Q2 — it fails open and silent if the local costmap is not publishing).
2. **Tracking-only test, DISARMED.** Full T4 stack up, `/follow/enable` left OFF. Watch `tools/follow_status.py` while a person walks around: detections steady, target range/bearing sane, `raw` stays empty because the controller is disabled. This is where the camera mount and the bearing get validated against reality rather than against a synthetic scan.
3. **First driven test**, user on the RC, `bash tools/pf_record.sh follow_walk_1`, preflight PASS first. User enables with `ros2 topic pub --once /follow/enable std_msgs/Bool "{data: true}"` and disables the same way with `false`.
4. Then: ground segmentation (P2), which is what makes low obstacles visible to the follow path too.

**Open questions for the driven test:** does the person stay inside the 55.7° FOV during turns (±1.58 m lateral at 3 m) — the FOV, not the controller, is the likely limit; and does the LiDAR keep a range lock on a person at 3 m through the `/scan_safety` 0.30–1.60 m band.

**Done by user 2026-09-25:** `ros-humble-vision-msgs` installed · D435 mount measured (0.99 m, level, below the LiDAR).

## Next session plan (written 2026-09-17 wrap-up) — superseded by the follow-me plan above
1. **Ground segmentation (P2):** build Patchwork++ (C++/ROS 2) on the Orin; measure CPU at 10 Hz. Offline on bags of this lane first. Output: non-ground cloud → costmap obstacle layer (+ `/scan_safety` if it helps near field). Static test: 15–25 cm box ahead stays in costmap as rover closes in. Live test: detour around it.
2. **P10 odom ZUPT** after hard stops (small, ROS-side).
3. **GPS waypoints in open sky vs Mission Planner AUTO (P6).**
4. Deferred user decisions: track imbalance `SERVO1_MAX` 1830 (P5), radio failsafe (P4), `COMPASS_ENABLE=1` intentional?
**Bring-up:** T1 `drive_bringup` → T2 `nav2_gps` (includes collision monitor) → T3 `foxglove` (no cloud while driving) → `tools/preflight.py` PASS → route check → recorder → user sends goal. Solo tests: `bash tools/solo_goal.sh <bag> "<goal args>" HH:MM:SS ...` (user runs it). Stop check: `tools/stop_distance.py`.

## Summary counts (2026-09-17)
- Part 2: ✅ Solved 33 · 🟡 Workaround/partial 3 · 🔴 Open 4 (P2, P5, P6 + P4 decision)
- **Biggest wins since July:** skid steer + demixer (J1) · dual-antenna GNSS heading (K0) · REP-105 fused odom (L1) · shim sampling distance = the oscillation fix (M3) · Fast-DDS profile = LiDAR back to 10 Hz (M1) · BODY_NED read-back safety gate (N1).

**H19 — Finding a process by a string that is also in MY command line** — `pgrep -f`, `kill $(pgrep -f …)`, and a /proc scan run from `bash -c "<script containing the pattern>"` all match the calling shell itself: killed my own shell once and made a CPU logger measure its own wrapper (loc1 CPU lost). *Fix:* hold PIDs from `Popen`, or put the script in a file and exclude own PID and ancestors.
