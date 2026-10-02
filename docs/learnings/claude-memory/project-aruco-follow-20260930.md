---
name: project-aruco-follow-20260930
description: ArUco-marker following (marker on a car/rover/anything) built on the click-to-follow path — design, measured limits, test status, what the live test still needs
metadata:
  type: project
---

User 2026-09-30: "in object following (person and aruco marker) we also have to do aruco following which can be mounted on anything (car, rover…); I have an aruco marker but no extra rover. Implement the same thing for aruco like person following."

**Built + committed 3e6d196 (report Q59), offline-tested end to end; NOT yet live.**
- Markers = second target kind on the same interface: `person_detector` runs OpenCV 4.5.4 ArUco on the same frame → `/follow/marker_detections`; new `marker_tracker` (+ ROS-free `marker_tracking.py`) → `/follow/marker_tracks` and, for the selected marker, `/follow/target` etc. Track id = 1 000 000 + marker id; person_tracker / marker_tracker each clear their selection when the other kind is clicked. Page draws cyan outlines, clickable. follow_controller / collision monitor / search / 1.6 m no-turn rule unchanged.
- Launch: `follow.launch.py marker_dictionary:=DICT_… marker_ids:=N marker_size:=0.xx` ("" dictionary = off); `detect_people:=false` = markers only.
- Rules: allowed ids, 3 sightings to confirm, ≥10 px / PnP fit ≤2 px / ≤75° tilt, motion gate, 2 sightings to resume; no LiDAR hold (v1); range from size or LiDAR when it agrees.
- Measured: 6.1 ms/frame on real frames; 4X4 dictionaries give single-frame false ids (all rejected by geometry), 5X5/6X6/7X7 none → recommend 5X5/6X6 if reprinting. OpenCV SUBPIX refinement (fixed 5 px window) biased range −3…−7 % at 3–5 m → own cornerSubPix (half a cell). Heading from a flat marker is ambiguous head-on → NaN unless unambiguous; not used for control. Range limit ≈ 6 m (15 cm), 8 m (20 cm), 12 m (30 cm) at 640×480.
- Tests: test_marker_tracking 34, test_marker_pipeline 14 (`--people` too), test_marker_follow 23, UI 15; person regressions green.

**Why:** user's goal is object following for both people and markers with the same click UI.
**How to apply:** NEXT = live: (1) user holds the marker 1.5–3 m in front, `python3 tools/marker_probe.py` → dictionary/id/size (user should also measure the black square with a ruler); (2) T4 with those args, tracking-only + `track_record.sh aruco1`; (3) driven test, user carries the marker, RC in hand. v2 ideas: LiDAR hold for markers, use carrier heading. **User's marker (live probe 09-30): DICT_6X6_50, id 0, 17.5 cm black to black** → launch `follow.launch.py det_conf:=0.2 marker_dictionary:=DICT_6X6_50 marker_ids:=0 marker_size:=0.175`; reads to ~6.5 m. Probe showed LiDAR 0.13 m behind the hand-held marker (body) → tracker now uses the marker's own range when ≥ 20 px, LiDAR only when smaller + agreeing; OpenCV minMarkerDistanceRate 0.05 drops 6x6 markers with a narrow white margin at close range → 0.03. marker_probe.py waits up to 180 s for the marker. v2 ideas: LiDAR hold for markers, use carrier heading. User's marker: about 17.5 x 17.5 cm (said 09-30; confirm it is the BLACK square, not the sheet) → readable to ~7 m; dictionary and id still unknown. marker_probe.py now waits up to 180 s for the marker (user has to walk to the rover). Related: [[project-click-to-follow-20260929]], [[project-deferred-tasks-20260930]], [[feedback-user-drives-physical-motion]].

## aruco1 live tracking-only test, 2026-09-30 14:24 (report Q60, commit c9f1e84)
- Recording `~/dagger_bags/track_aruco1_142439`. Marker read 2.2–8.5 m, no false marker, selection switching person↔marker OK, SEARCHING→TRACKING OK, range within 3 cm of LiDAR at 4 m (chest-held).
- Fault: at 2.2–4 m (following distance) missed 63/175 frames walking — the print's white margin is only 1.3–2.0 cm and the shirt beside it thresholds dark. Fixed with a throttled second detection pass (adaptiveThreshConstant 25, win 53): 170/175, max gap 0.3 s, 0 false in 3332 marker-free frames, +1.4 ms idle. `marker_ids` now also passed to person_detector.
- Told the user: remount the marker on a white board with ≥ 3 cm white margin.
- NEXT: **driven test** (`track_record.sh aruco2`), user carries the marker, preflight PASS, RC in hand. Not yet verified live: the second pass on raw (non-JPEG) frames while following.
- Pitfall seen: a session interrupted mid-tool-call can keep writing files for a minute after the next session starts — re-check `git status` before editing, and review any uncommitted diff found.

## aruco2 DRIVEN test, 2026-09-30 15:00 (report Q61, commit 322da9c)
- `~/dagger_bags/track_aruco2_150009`. User: "seems good for now". Follows at ~0.6 m/s, stops 3.1 m behind the marker, holds still inside 1.6 m, 474/480 frames read while followed, all 7 searches were real losses of sight and recovered.
- Bug fixed (offline-tested, NOT yet re-tested live): CLEAR while following → controller searched 29 s (736° turn). follow_controller now listens to /follow/select: -1 or another id → stands down at once.
- Open: (1) **LiDAR hold of the carrier** — marker hidden with the carrier still in front → 3 s hold then 333° sweep; recommended next step. (2) second detection pass not exercised live (evening light). (3) remount marker with ≥3 cm white margin. (4) FCU speed overshoot 0.74 m/s on the ramp; odometry slides 0.03–0.19 m backward while standing (LiDAR says no motion) — report P16.
- track_record.sh now records /cmd_vel_follow_raw, /cmd_vel_follow, /follow/enable.

## Wrap-up 2026-09-30
Pushed to GitHub (652cd4e). Report has "Next session plan — following (written 2026-09-30 wrap-up)": 1) re-test CLEAR live, 2) LiDAR hold of the carrier, 3) white board ≥3 cm margin + second pass in sunlight, 4) parked items. Command reference `ros2_ws/COMMANDS.md` must be kept updated ([[feedback-keep-commands-doc-updated]]).
