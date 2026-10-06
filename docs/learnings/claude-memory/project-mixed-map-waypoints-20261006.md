---
name: project-mixed-map-waypoints-20261006
description: "2026-10-06 — Foxglove waypoint routes live (Q81); one indoor+outdoor 2D map mixed1 (Q82) saved, georeferenced, self-replay OK; live localization on it is NEXT"
metadata:
  node_type: memory
  type: project
  originSessionId: 6b6241ab-7593-43ac-82a4-bd647ef0289d
  modified: 2026-10-06T06:32:00.822Z
---

**2026-10-06 state (pushed at the wrap-up):**
- **Q81 waypoint routes from Foxglove — live OK** (operator "worked good", bag not analysed): `waypoint_queue` (started by foxglove.launch.py): pose tool on `/waypoint_ui` adds waypoints, Publish `/waypoints/go` drives them (NavigateThroughPoses), `/waypoints/clear` resets; odom routes use `dagger_navigate_through_poses_odom.xml`.
- **Q82 map `mixed1`** (one continuous RC drive, outdoor → door → indoor hall → back out, 929 s / 161 m, loop closed): saved (`map_saver_cli` needs `-p save_map_timeout:=20.0`), georeferenced with 1/σ² GNSS weighting — the GNSS at this site is only good to 3-7 m, so lat/lon goals on mixed1 carry ~3-5 m; odom/map-frame clicks don't.
- Operator saw "two walls" on the long lower-right wall → **real stepped structure** (low face 0-0.4 m sits 0.25 m in front of the wall; /scan's 0.12-0.75 m band hits the low face from ≲10 m, the upper wall from ≳11 m). Not a pose error — don't "fix" it.
- Offline localization self-replay on mixed1: seeded + localized, score median 0.038 m, driving corrections median 0.05 / max 0.99 m.

**Why:** the user wants one map spanning indoor and outdoor, driven from one continuous run (2D, Nav2 consumes 2D).
**How to apply:** next = first LIVE localization drive on mixed1, starting OUTDOORS (slam_geo_init searches ±8 m around GNSS): T2 `map_source:=slam`, T5 `mode:=localization map:=mixed1`; then goals / a waypoint route across the door. Related: [[project-lidar-odometry-20261003]], [[feedback-dont-over-engineer]].
