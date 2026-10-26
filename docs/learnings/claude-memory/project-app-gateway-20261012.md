---
name: project-app-gateway-20261012
description: "Operator-app gateway dagger_gateway (ws://:8090/api, protobuf contract submodule proto/) — steps 1-6 built offline 2026-10-12; live steps 7 (real STOP) + 8 (systemd install) pending the owner"
metadata:
  node_type: memory
  type: project
  originSessionId: c3b71c56-c31f-4d21-a85a-df5b0ef87725
  modified: 2026-10-12T05:33:39.483Z
---

2026-10-12: built `src/dagger_gateway` from the user's pasted spec (DaggerGCS app ↔ rover; contract = github AbhinavGarg815/dagger_proto as git submodule `proto/`). Report Q83 + open item P21. Commits 37b148d … c8e5539 (local, not pushed).

- Protobuf decision: apt protobuf-compiler 3.12.4 + `--experimental_allow_proto3_optional` (user installed it); byte-identical on all 19 vectors. setup.py runs tools/gen_proto.sh each build.
- Decisions user OK'd: any `/estop false` releases the gateway's STOP latch (it streams true at 10 Hz while latched, else estop_clear.sh would be overridden); app-silence follow-off applies to follow started anywhere, handshaken apps only, disconnect = silent at once; speed <0.05 → 0; RUNNING_ROUTE only for gateway routes (Phase 3).
- Added `fcu_config` node to T1 (re-applies BODY_NED + stream rates on every FCU connect) — needed for boot without preflight.
- Offline: test_gateway_ros.py 44/44 (domain 42), contract_test 29/29, test_rover_state 7/7, test_fcu_config PASS.

**Why:** Phase 1 exit test = contract_test passes against the real gateway with the rover parked.
**How to apply:** next: live steps 4-6 on the real stack (check gateway CPU — raw /rslidar_points sub), then step 7 the OWNER runs (rover parked, RC in hand; see [[feedback-user-drives-physical-motion]]), step 8 `sudo bash tools/systemd/install.sh` + reboot. Known gaps: indoor readiness (RC10 ExtNav → POSITION/HEADING FAIL), map→odom owner check. Related: [[project-siyi-mk32-ui-20261002]].
