---
name: project-obstacle-avoidance-next-20261002
description: Next session = obstacle-avoidance gaps (near-field blind zone P13, low obstacles P14, live detour test); hand-off prompt location and status estimate at hand-off
metadata:
  type: project
---

User 2026-10-02: "lets continue with obstacle-avoidance gap, but we will do this in new session" — asked for a hand-off prompt.

- Prompt written to `/home/jetson/dagger_reports/next_session_prompt.md` (replaces the 09-25 one). Report section "Next session plan — obstacle avoidance (written 2026-10-02)".
- Order: (1) P13 near-field blind zone of /scan_safety (range_min 1.0 for mast self-returns; footprint corners sweep 1.03 m) → raw-cloud capture parked, body filter (self_filter.py exists, unused) + range_min ~0.3; NEEDS USER OK (safety config); re-check everything reading /scan_safety (both collision monitors, watchdog, follow slow zone, person_tracker min_range/near_range 1.0, marker_tracker min_range 0.3). (2) P14 low obstacles erased inside ~3.3 m — shelved low_obstacle_layer failed on people leaving blobs; ideas: ±2-ray tall check, N-scan persistence per odom cell, clear only observable cells. (3) Live detour test, placement from beam geometry (LiDAR 1.12 m, lowest beam −15°). (4) later: avoidance on the follow path.
- Status estimate given to the user: object following ~80 %, SLAM ~70 %, waypoint following ~70 %, obstacle avoidance ~55 %, road route 0 %.

**Why/How:** start the next session from this; user wants a plan before implementation. Related: [[project-deferred-tasks-20260930]], [[feedback-user-drives-physical-motion]], [[feedback-keep-commands-doc-updated]].
