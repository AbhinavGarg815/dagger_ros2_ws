# Dagger — ArduRover (Cube Orange) Config Checklist

> **⚠️ HISTORICAL (written July 2026).** Steps 1–3 were done: skid steer 73/74 + STM32 demixer.
> Other items were superseded: RC override/ACRO dropped, compass→dual-antenna GNSS yaw (`EK3_SRC1_YAW=2`),
> `SYSID_MYGCS` back to 255. **Current verified FCU config + pending changes: see
> `issues_and_fixes.md` → "Current configuration (verified 2026-09-17)" and N6/P4.**
>
> ### Pending FCU changes (user decides / sets)
> | Param | Now | Proposed | Why | Status |
> |---|---|---|---|---|
> | `SERVO1_MAX` | 1900 | 1830 | left track needs ×0.82 range to match right (N6) | deferred 2026-09-17 |
> | `FS_THR_ENABLE` | 1 | decide | TX dead → "PreArm: Radio failsafe on" (P4) | open |
>
> ### FCU changes since July (all set by user)
> | Param | From → To | Date | Ref |
> |---|---|---|---|
> | `SERVO1/3_FUNCTION` | 26/70 → 73/74 | 07-24 | J1 |
> | `ATC_STR_RAT_FF` | 0 → 0.8 | 07-25 | E4 |
> | `EK3_SRC1_YAW` | 8 (GSF) → 2 (GPS) | 09-11 | K0 |
> | `ATC_STR_RAT_I` / `P` | 0.05/0.2 → 0.2/0.3 | 09-14 | L7 |
> | `ATC_STR_RAT_IMAX` | 0.4 → 0.2 | 09-14 | L8 |


Goal: make the tracked rover move correctly in MANUAL/ACRO/GUIDED/AUTO, fix reverse,
and restore a reliable TX (hardware E-stop). Based on the ROS-side diagnosis
(current setup is steering+throttle/Ackermann, which is why GUIDED can't pivot).

Mission Planner locations: **SETUP** = Setup tab, **CONFIG** = Config → Full Parameter List.
Change params in Full Parameter List → **Write Params** after each group. Some need a reboot.

---

## STEP 0 — Back up first
- CONFIG → Full Parameter List → **Save to file** (so you can revert). Note current values of
  anything you change.

## STEP 1 — Decide the drivetrain type (the crux)
The vehicle currently runs **SERVO1_FUNCTION=26 (GroundSteering) + SERVO3_FUNCTION=70 (Throttle)**
into one mixer. GUIDED can't pivot a "car." Find out which is true:
- **Can the two tracks be driven independently** (two ESC inputs, or the controller has a
  left/right / "independent" mode)? → do **STEP 2 (skid steering)** — the real fix.
- **Only a steering+throttle mixer, can't split?** → skip STEP 2; GUIDED will only arc, and the
  ROS RC-override path stays the better option. (Consider a controller that accepts L/R.)

## STEP 2 — Convert to SKID STEERING (only if tracks can be driven independently)
In Full Parameter List, set the two outputs that go to the LEFT and RIGHT tracks:
- **SERVO1_FUNCTION = 73**  (ThrottleLeft)
- **SERVO3_FUNCTION = 74**  (ThrottleRight)
  - (use whichever physical outputs are wired to each track; 73=left, 74=right)
- **SERVO2_FUNCTION = 0**, **SERVO4_FUNCTION = 0** (unused)
- **PILOT_STEER_TYPE = 0** (default two-paddle mixing for manual)
- Reboot. In skid mode ArduRover does **in-place pivots natively** → GUIDED/AUTO turns work.

## STEP 3 — Motor Test (verify wiring + direction + reverse)  ← do this before driving
SETUP → Optional Hardware → **Motor Test** (wheels off ground / rover on blocks):
- Test **Left** track forward → should spin the left track **forward**. Then reverse.
- Test **Right** track forward → right track forward. Then reverse.
- If a track runs backward: flip its **SERVOx_REVERSED** (0↔1).
- If left/right are swapped: swap the two **SERVOx_FUNCTION** (73↔74).
- Confirm **reverse actually works** (track spins backward). If not → it's the ESC/controller
  (needs reverse enabled / bidirectional), not ArduPilot.
- Ranges for full bidirectional travel: **SERVOx_MIN=1000, SERVOx_TRIM=1500, SERVOx_MAX=2000**.
- **MOT_PWM_TYPE**: match your ESC (0=Normal PWM is typical; set if the controller needs a
  specific protocol). **MOT_THR_MIN=0**, **MOT_SLEWRATE** ~100 (lower = gentler ramp).

## STEP 4 — RC / TX + failsafe (so the TX stops "ending")
- SETUP → Mandatory Hardware → **Radio Calibration** → recalibrate the transmitter fully.
- Check the **receiver link / binding** and **RC_PROTOCOL** (auto is fine unless a specific RX).
- Failsafe (CONFIG):
  - **FS_THR_ENABLE = 1** (enable throttle/RC-loss failsafe; it was 0 = none)
  - **FS_ACTION**: 1=Hold (recommended for testing) or 2=RTL (needs GPS)
  - **FS_TIMEOUT** ~1.5 s, **FS_CRASH_CHECK** as desired
- Set a **TX switch to a mode / emergency** (e.g., a switch → HOLD) so the TX is a real E-stop.
- **ARMING_CHECK**: leave sensible checks on; if it blocks arming, read the message and fix the
  specific item rather than disabling all checks.

## STEP 5 — Modes & GUIDED
- SETUP → Flight Modes: assign a TX switch to **MANUAL / HOLD / GUIDED (or AUTO / ACRO)**.
- GUIDED needs a **position estimate**:
  - Outdoors: GPS + good HDOP. Verify **GPS 3D fix** and **EKF green** (HUD) before GUIDED.
  - Indoors (no GPS): GUIDED position control won't work; only heading/rate modes.
- **Speeds/turn:** CRUISE_SPEED, WP_SPEED (AUTO), **WP_PIVOT_ANGLE** (angle above which it pivots
  in place, skid only), **WP_PIVOT_RATE** (deg/s) — set a moderate pivot rate for the heavy rover.
- **TURN_MAX_G / ATC_TURN_MAX_G**: keep modest so it doesn't demand impossible turn rates.

## STEP 6 — Steering-rate controller (retune for the new config)
After skid conversion the steering loop changes:
- Reset **ATC_STR_RAT_P / _I / _D / _FF** toward Rover defaults (P=0.2, I=0.2, D=0, FF=0.2),
  then use **ACRO** mode to hand-tune, or Rover **AUTOTUNE** if available.
- **ATC_STR_RAT_MAX** (max turn rate, deg/s) and **ATC_STR_ANG_P** (heading→rate) sensible.

## STEP 7 — Reset our ROS-debug changes (if going pure-GUIDED)
We changed these while debugging via MAVROS/RC-override — decide per your final GCS setup:
- **SYSID_MYGCS**: was 255, we set **1** for MAVROS RC-override. Keep **1** if the Jetson/MAVROS
  will send commands; set back to 255 if only Mission Planner drives. (RC override / some GCS
  actions require the sender's sysid to match this.)
- **ATC_STR_RAT** gains and **ACRO_TURN_RATE=57** were our tuning — reset/retune per STEP 6.
- **SERVO1_REVERSED** currently 1 (correct for the TX). Re-verify in Motor Test after any change.

## STEP 8 — Test order (safe ramp-up, wheels-off first)
1. Motor Test (STEP 3) — each track, both directions.
2. **MANUAL** with TX — forward/back/left/right/pivot all correct.
3. **ACRO** — steady turn-rate tracking (retune ATC_STR_RAT if it oscillates).
4. **GUIDED** (outdoors, GPS/EKF green) — send a position + heading goal; confirm it pivots &
   drives to it.
5. Only then bring the Jetson/Nav2 back on top (MAVROS GUIDED **or** the RC-override path).

---

### Quick reference — key params
| Param | Set to | Why |
|---|---|---|
| SERVO1_FUNCTION | 73 (ThrottleLeft)* | skid steering (*if independent L/R) |
| SERVO3_FUNCTION | 74 (ThrottleRight)* | skid steering |
| PILOT_STEER_TYPE | 0 | manual mixing |
| SERVOx_MIN/TRIM/MAX | 1000/1500/2000 | full bidirectional |
| FS_THR_ENABLE | 1 | TX-loss failsafe (was 0) |
| FS_ACTION | 1 (Hold) | safe on RC loss |
| WP_PIVOT_ANGLE / _RATE | e.g. 60 / moderate | in-place pivots in AUTO (skid) |
| ATC_STR_RAT_P/I/D/FF | 0.2/0.2/0/0.2 then tune | steering-rate loop |
| SYSID_MYGCS | 1 (MAVROS) or 255 (MP) | command source gate |

**Biggest lever:** STEP 2 (true skid steering). If the hardware allows it, GUIDED turns become a
non-issue. If it doesn't, GUIDED can only arc — and the ROS RC-override drive we built is the fallback.
