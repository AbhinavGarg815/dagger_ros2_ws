---
name: feedback-user-drives-physical-motion
description: "On the physical rover, the USER initiates motion commands; Claude monitors, does not auto-publish motion"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 4d48c96a-557d-4105-b251-751ed27dee94
---

When testing the physical Dagger rover, the **user initiates the motion commands** (teleop / `ros2 topic pub` cmd_vel); Claude should **monitor and read traces, not auto-publish motion**. The user interrupted a Claude-issued forward-drive `ros2 topic pub` (2026-07-12), especially relevant now that the **RC transmitter is dead** so the only stop is the laptop E-stop.

**Why:** safety — with no working TX, the user wants to own every command that moves a ~350 kg vehicle and keep a hand on the laptop E-stop.

**How to apply:** set up monitors, give the user the exact command to run, then read the resulting trace. Only publish motion myself if the user explicitly says so. Laptop E-stop is `/tmp/estop.sh` (HOLD + `/estop true`, latched) and release is `/tmp/estop_clear.sh` (`/estop false` + MANUAL) — see [[dagger-outdoor-ugv-project]].

**UPDATE 2026-09-17:** user now has a **working RC transmitter in hand** as the E-stop ("dont need laptop estop"). Rule unchanged: the user still issues all motion; Claude monitors. Don't insist on the laptop E-stop scripts in test instructions — mention RC takeover instead.
