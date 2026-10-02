---
name: project-deferred-tasks-20260930
description: "Tasks the user explicitly parked for the future on 2026-09-30 (after click-to-follow reid3), before starting ArUco following"
metadata:
  node_type: memory
  type: project
  originSessionId: 5c15c34f-dbac-4cc2-816f-8b5cfc96d79b
  modified: 2026-09-30T07:43:24.726Z
---

User 2026-09-30: "Keep these tasks for future" — do NOT start them unprompted; offer them when the current work (ArUco following) is done:

1. Near-field blind zone of /scan_safety (range_min 1.0 m) for both collision monitors — raw-cloud capture parked → self-filter box + range_min ~0.3. Safety config: needs user OK. (report P13)
2. Click-to-follow crowd test with 2–3 helpers (crossing, occlusion, look-alike) + end-to-end video latency measurement. (report P12)
3. ~~Push to GitHub~~ DONE 2026-09-30 (652cd4e) and 2026-10-02 (15206ff), each on the user's request. Still ask before each later push unless told otherwise.
4. Nav2 low-obstacle detour ("task 2"); goal tolerance ("fine for now"). (report P14)
6. Added 2026-10-02 (user: "for now it's fine for latency and CPU but keep in future tasks"): handset video latency/CPU options — try LITE at range (MK32 decode share of its ~45 ms), D435 colour at 60 fps (~15 ms less camera delay, more USB/CPU — untested), physical MK32 switch → RC channel → /mavros/rc/in for follow STOP/FOLLOW. (report P17)
5. Parked: GPS waypoints open sky vs Mission Planner (P6), FCU items (FS_THR_ENABLE, SERVO1_MAX, COMPASS_ENABLE), KISS-ICP, p2l wedge root cause.

**Why:** user wants to switch to ArUco-marker following next and keep these for later.
**How to apply:** when ArUco following reaches a stopping point, list these as the next candidates. Related: [[project-click-to-follow-20260929]].
