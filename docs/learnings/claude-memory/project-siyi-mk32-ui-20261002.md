---
name: project-siyi-mk32-ui-20261002
description: Follow page on the SIYI MK32 handset over the SIYI radio link — network facts, what was built (24eaf30, report Q62), what is pending
metadata:
  type: project
---

User 2026-10-02: "can we get our interface on siyi mk32 rc instead of phone/laptop? jetson is connected to lan of receiver … very low latency feed, make a plan if possible and start implementing."

**Network (measured):** Jetson `enP8p1s0` = 192.168.144.100/24 (100 Mb/s) on the SIYI LAN. Air unit .11 (0.3 ms), ground unit .12 (1.9 ms), **MK32 Android .20** (4.5 ms mean, 7.8 max, port 5555 adb open — not used, the user's device), .33 local wired device (likely LiDAR). Page URL on the handset: `http://192.168.144.100:8080`.

**Built 24eaf30 (offline-tested, NOT yet seen on the MK32):** handset layout (touch → big button column, FULL = fullscreen), tap tolerance, no DOMMatrix/createImageBitmap dependency, per-page LITE video (320×240 ~10 KB, 2.4 ms), deadman now tied to the page that pressed FOLLOW (was: any page), `tools/ui_latency.py`.

**Pending:** (A) user opens the page on the MK32 + reads age/fps/rtt; `ui_latency.py` baseline on the Jetson; (C) trim the Jetson-side delay where measured (marker detection + UI JPEG run after YOLOX before the frame can leave; option: send video before boxes); (D) optional physical MK32 switch → RC channel → /mavros/rc/in → follow STOP/FOLLOW (needs the user's channel choice; FCU RCx_OPTION must not clash). Rejected: H.264 to SIYI FPV app (no NVENC on Orin Nano, no click).

**Why/How:** user wants the operator interface on the RC handset; keep the browser page as the single UI. Related: [[project-click-to-follow-20260929]], [[project-aruco-follow-20260930]], [[feedback-keep-commands-doc-updated]].

## Live 2026-10-02 + video first (15206ff, report Q63)
- MK32 live: page works, age ~150 ms, 10 fps, RTT 10 ms; Jetson camera→sent 100 ms (ui_latency). MK32 browser: no `pointer:coarse` (handset now by touch support), no flex `gap` (use margins — keep it that way).
- Stage probe: camera 40 ms, detections 92–94 (YOLOX 38 + markers 12 in series), tracks 97, JPEG 99 → video waited 55 ms for detection.
- Built VIDEO FIRST: detector encodes JPEG + runs markers in a worker thread during YOLOX; ui_server sends frames at once, boxes as separate {"type":"boxes"} text messages; page redraws boxes on their frame. Handset UI redesigned (state tile, big STOP column).
- NOT yet verified live: user must restart T4 + reload the page; then `python3 tools/ui_latency.py` (expect camera→sent ~45 ms, boxes ~100) and the handset age (expect ~95 ms). `test_marker_pipeline.py --people` (JPEG before detections) still to run with T4 down (1.2 GB free with stack up — don't load a 2nd detector beside it).
- ✅ VERIFIED LIVE 2026-10-02 after T4 restart: camera→sent 51.5 ms (was 100.5), boxes 97 ms, MK32 age ~100 ms (was 150), detection + selection work from the handset. Remaining options (user's call): LITE at range; physical MK32 switch for STOP/FOLLOW; camera 60 fps (untested, CPU cost).
