---
name: reference-dagger-monitoring-tools
description: Cross-user DDS fix (UDP-only) + chain_monitor.py needed to observe MAVROS topics on the Dagger Jetson
metadata: 
  node_type: memory
  type: reference
  originSessionId: 4d48c96a-557d-4105-b251-751ed27dee94
---

To monitor MAVROS/ROS2 topics on the Dagger Jetson **as user `nvidia` while the stack runs as `jetson`**, you MUST force UDP-only DDS transport, or every topic shows a publisher but delivers **zero messages** (and `ros2 topic echo` just hangs):

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/dagger_reports/tools/udp_only.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**Why:** FastDDS's default shared-memory transport uses `/dev/shm/fastrtps_*` segments owned by `jetson`. Discovery happens over UDP multicast (so `ros2 topic list` and `topic info` look healthy, "Publisher count: 1"), but a cross-user subscriber can't attach to those SHM segments. Symptom is easily mistaken for a QoS mismatch — it isn't. (Note `/mavros/rc/out` and `/mavros/imu/data` *also* genuinely need `--qos-reliability best_effort`.)

**Tools** (kept in `/home/jetson/dagger_reports/tools/`, durable — `/tmp` gets wiped mid-session, see [[feedback-git-and-issues-report]]):
- `udp_only.xml` — the DDS profile above.
- `chain_monitor.py <out.csv>` — samples the whole command chain at 10Hz onto ONE timestamped CSV: teleop → twist_mux → cmd_vel_safety → setpoint → `/mavros/rc/out` PWM → IMU, each with an **age** column. Use this instead of separate `ros2 topic echo` logs — correlating async streams by line number gives false conclusions.

**How to apply:** verify `mode` in the CSV immediately before any run (the FCU drifts out of GUIDED — see [[dagger-outdoor-ugv-project]]), and remember the user drives all motion ([[feedback-user-drives-physical-motion]]).

---
**Update (2026-07-24): Claude Code now runs AS the jetson user (migrated from nvidia).** The cross-user UDP-only DDS workaround above is **no longer needed** when running as jetson — same user as MAVROS, so native shared-memory transport works and topics deliver normally. `sudo -u jetson` bridging is also obsolete. Keep `udp_only.xml` only in case a cross-user session ever recurs. Tools now live at /home/jetson/dagger_reports/tools/.

---
**Update (2026-09-14):** Claude is `nvidia` again and the ROS stack now also runs as `nvidia` → native SHM, no bridge. New tools live in the tracked `/home/jetson/ros2_ws/tools/` (preflight.py, pf_record.sh, pf_analyze.py, send_goal.py) — see [[project-pathfollow-20260914]]. The /tmp/claude-1001 copies (light_log.py, udp_only.xml) were wiped; the durable udp_only.xml is still in /home/jetson/dagger_reports/tools/.
