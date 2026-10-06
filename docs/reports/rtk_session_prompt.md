# Dagger UGV — RTK set-up session prompt (written 2026-10-06)

Paste everything below the line into a new session.

---

Help me set up **RTK GNSS** on the Dagger UGV with my own base station: what to configure, wire, add or
remove, and the code. Dagger is a 350 kg tracked rover that works outdoors and indoors.
- **Software:** ROS 2 Humble on a Jetson Orin Nano; ArduRover 4.6.3 on a Cube Orange+ over MAVROS 2.14.
- **Sensors:** RS-Helios-16P LiDAR, D435.
- **Workspace:** `/home/jetson/ros2_ws` (git, private remote github.com/AbhinavGarg815/dagger_ros2_ws).

**Another Claude session may be working in the same repo.** Run `git status` and `git log -5` before
editing, commit in small steps, and never commit or revert files you did not change.

**Plan first, get my OK, then build.** Recommend one option and say why; don't list every alternative.

## Hardware
- **Base (on the ground, fixed):** u-blox ZED-F9P and an RFD868ux-IND radio.
- **Rover GNSS:** CUAV **C-RTK 2HP**, dual antenna with moving-baseline heading.
  - On the Cube's **CAN1**: DroneCAN, `GPS1_TYPE 9`, antenna offsets `GPS1_POS_X/Y 0.33/0.29`.
- **Rover radio:** RFD868ux-IND, installed 2026-09-11 on the Cube's **TELEM2** (`SERIAL2_PROTOCOL 2`,
  MAVLink2) "for RTK corrections later". Confirm with me that this is the rover radio I mean, and
  whether a ground radio for a GCS also talks to it.
- **Jetson to Cube:** USB, `/dev/serial/by-id/usb-CubePilot_CubeOrange+_…-if00`.

## Why, and what depends on GNSS today
- **GNSS today:** 3D fix only, no RTK.
  - h_acc is 2-7 m outdoors at the mapping site (1.1 m on the best day) and 4-17 m indoors.
  - The dual-antenna heading is good: about 0.5°, valid 96-100 % of the time outdoors.
- **The heading must keep working.** `EK3_SRC1_YAW 2` uses it, and so does the odometry EKF
  (`/gps/heading_imu`).
- **What the position error costs now** (see `docs/reports/issues_and_fixes.md`):
  - **map→odom walks.** `gps_map_odom` (map→odom in GPS mode) moved 0.68 m in 27 s parked and
    2.55 m in one session (Q69). Its input is low-pass filtered and gated at `max_h_acc 5 m`.
    Because of the walk, Foxglove goal clicks must be made in `odom`.
  - **Lat/lon goals** (`tools/gps_goal.py`, Q49-Q50) carry a 2-5 m bias.
  - **SLAM maps** are georeferenced from GNSS (`tools/slam_georef.py`, weighted by 1/σ²); the
    indoor + outdoor map `mixed1` is only good to about 3-5 m (Q82). `slam_geo_init` searches ±8 m
    around the GNSS position to seed localization (`max_h_acc_m 6`).
  - **ArduPilot's own EK3 position** (source set 1 = GPS) walks 1-3 m/min.
- **Goal:** RTK FIXED outdoors (cm level). After that, only follow-ups that measurably win:
  re-georeference the maps, tighten the gates, use map-frame and lat/lon goals. Nothing changes
  indoors (LiDAR odometry and ExternalNav, Q80).

## Already there
- **MAVROS `gps_rtk` plugin** (mavros_extras 2.14):
  - It takes `mavros_msgs/RTCM` on `/mavros/gps_rtk/send_rtcm` (at most 720 bytes per message) and
    fragments it into `GPS_RTCM_DATA`.
  - It is **not** in T1's plugin allowlist (`minimal` in
    `src/dagger_bringup/launch/drive_bringup.launch.py`); the comment there says "add when needed:
    gps_rtk".
- **Fix-type names:** `tools/preflight.py` already names 5 = RTK_FLOAT and 6 = RTK_FIXED.
- **Not installed:** pyserial, pyubx2 and PyGPSClient. There is no u-center on the Jetson (Linux
  arm64). Install pip packages into a venv, not system-wide.

## Decide: research, check the current docs, then recommend one
1. **Correction path.** ArduPilot forwards corrections to a DroneCAN GPS from MAVLink
   `GPS_RTCM_DATA`. For 4.6.3 and the 2HP, verify whether a raw RTCM stream arriving on a serial
   port is used at all; I believe a MAVLink port does not parse raw RTCM. Candidates:
   - **A — the previous session's leaning.**
     - Base F9P UART → base RFD868, sending raw RTCM3 (transparent).
     - Rover RFD868 moved to a **Jetson USB-serial adapter**.
     - A small ROS 2 node reads the serial port, frames RTCM3 by preamble, length and CRC24Q, and
       publishes `mavros_msgs/RTCM`.
     - mavros `gps_rtk` → Cube (USB) → CAN → 2HP.
     - Pros: no laptop at the base; corrections are logged in bags; preflight can show their age.
   - **B.** Base F9P on a laptop running Mission Planner (RTK inject) → MAVLink over the RFD pair →
     TELEM2. Needs a laptop at the base for every drive.
   - **C.** Rover radio wired straight into the 2HP. Only if the 2HP accepts corrections on its own
     port; check CUAV's docs.
2. **Base position.** If the base runs a survey-in every day, its absolute position shifts by the
   survey accuracy each time, and every georeferenced map shifts with it. Recommend how to get one
   fixed base coordinate: a long survey-in once, a known mark, or PPP. Then re-use it in fixed
   mode, with the antenna on the same mark every time.
3. **RTCM messages and bandwidth.** Which RTCM3 message set the 2HP needs: 1005, plus MSM for the
   constellations both receivers track, plus 1230. Check that it fits the RFD868 air rate.
4. **Radio settings** (SiK parameters): serial and air speed, NETID, ECC, MAVLink framing off for
   raw RTCM, and the IND variant's band, LBT and duty cycle. Keep the India band and power within
   the IND limits.
5. **FCU parameters** the chosen path needs: for example GPS inject, auto-config or driver options,
   and `SERIAL2_*` if the radio leaves TELEM2. Read the current values first, confirm the names
   exist on 4.6.3, and don't break the moving-baseline heading.

## Build (after I OK the plan)
- **Jetson side.** What the chosen path needs, for example:
  - `gps_rtk` added to the T1 allowlist;
  - the RTCM serial node in `dagger_bringup`, with an offline test (recorded RTCM bytes in →
    messages out, CRC checked);
  - a launch argument, off by default until proven;
  - preflight showing the RTK state and the age of the corrections;
  - the new topics added to `tools/detour_record.sh` and `tools/slam_record.sh`.
- **Verification, in this order:**
  1. Base output on the bench: RTCM frames come out with clean CRCs.
  2. Radio link on the bench: bytes and frame loss over the air.
  3. Rover parked outdoors in open sky: time to RTK FLOAT and FIXED, h_acc, 10 min of position
     scatter, and the heading still valid.
  4. A drive recorded alongside the LiDAR odometry for comparison.
- **Follow-ups, only after verification:** for example re-georeference `lane3fix` and `mixed1` with
  RTK, or tighten `gps_map_odom` and `slam_geo_init`. Make each the smallest change the data supports.

## Read first
- **`docs/reports/issues_and_fixes.md`:**
  - K0 (2HP install), Q49-Q50 (lat/lon goals), Q69 (the map→odom walk), Q82 (the `mixed1` georeference);
  - the "Current configuration" and "Open items" sections (P6: GPS goals in open sky).
- **`COMMANDS.md`:** sections 3 (bring-up), 6 (SLAM) and 12 (FCU parameters).
- **Code:**
  - `src/dagger_bringup/launch/drive_bringup.launch.py`;
  - in `src/dagger_bringup/dagger_bringup/`: `gps_map_odom.py`, `slam_geo_init.py`, `gps_heading_imu.py`;
  - `tools/preflight.py` and `tools/slam_georef.py`.

## Ask me first
- **The base F9P board:** a bare module or a carrier board, and does it break out USB and UART2?
- **The base antenna** and how it is mounted.
- **Base power.**
- **A laptop at the base:** whether one can stay there during drives.
- **The rover radio's** exact connection.
- **Base location:** where on the site the base can sit with open sky.
- **A surveyed point:** whether I have one.

## Standing rules (from me, still in force)
- **I drive all motion.** Never send goals, arm, change modes or publish velocity. Before each test,
  give me the exact command and say what the rover will do (straight, turn left, stop at …).
- **FCU parameters are mine.** Read them with a single GetParameters call. For writes, give me
  `ros2 param set /mavros/param <NAME> <value>` together with the old value to undo.
- **Hardware and firmware are mine too.** No flashing (F9P, RFD868, 2HP, Cube) and no rewiring;
  give me the steps.
- **Safety configuration changes need my OK before you apply them.** That covers collision monitor
  zones, `/scan_safety` and anything else the stop path depends on.
- **I start the terminals:** T1 `drive_bringup`, T2 `nav2_gps`, T3 `foxglove`, T4 `follow`, T5 `slam`.
  Before any drive, `python3 tools/preflight.py` must PASS. The RC transmitter in my hand is the E-stop.
- **Never pick a test line, heading or goal from the costmap alone.** It cannot see drop-offs; ask
  me what bounds the site.
- **While I run a timed test, don't poll or block.** Wait for "done", then read the bag. Stop the
  recorder before reading a bag.
- **Update `docs/reports/issues_and_fixes.md` and `COMMANDS.md` in the same session** as each fix,
  finding or new command. Use the next free Q number.
- **Git:** commit as Abhinav with **no co-author trailer**. At wrap-up, run
  `bash tools/sync_learnings.sh`, commit and push. Ask before pushing at other times.
- **Kill by PID**, never `pkill -f <pattern>`. List before killing, because a `pgrep -f` pattern also
  matches the shell that runs it.
- **Don't over-engineer.** Fix measured problems with the simplest change. If the effect is small,
  record it and move on. If a fix doesn't clearly win, revert it rather than stacking another layer.
