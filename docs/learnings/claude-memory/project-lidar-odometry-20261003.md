---
name: project-lidar-odometry-20261003
description: P20/Q73 LiDAR odometry — VGICP 0.5 m selected by 6-drive benchmark; C++ lidar_odom built but diverges on replay (open bug); next steps
metadata:
  type: project
---

**Why:** `/odometry/filtered` velocity = FCU EK3 (GNSS velocity); under degraded GNSS (lane end, h_acc 3-4 m, heading invalid) it said stopped/backing while the rover drove 1.1 m (Q72, bag detour_fox3_144415). User wants LiDAR odometry that works indoor + outdoor ([[dagger-outdoor-ugv-project]]).

**Selected 2026-10-03 (Q73):** small_gicp VGICP scan-to-model, 0.5 m voxels, LRU 100/10, 0.2 m downsample, k=10, constant velocity, no deskew. Benchmark tools/lo_bench.py + lo_eval.py (venv with pip kiss-icp/small_gicp/genz-icp; OMP/OPENBLAS threads = 1 or CPU numbers double). KISS-ICP lost (pivot yaw errors, jitter); GenZ-ICP lost (CPU spikes >1 s); IMU-based LIO impossible (Cube caps IMU at 10 Hz); 10 Hz raw-IMU gravity alignment made tilt WORSE.

**State:** C++ `lidar_odom` (dagger_perception) + dagger_msgs/LidarOdomStatus built, NOT launched. Replay of fox3 matches Python scan-for-scan ~35 scans then the CV guess creeps and diverges; Python stable. Ruled out: Eigen, cov setter, optimizer state, -march, float transform. `debug:=true` per-scan log. tools/test_lidar_odom.py synthetic 6/8.

**How to apply:** next session = fix that bug first, then covariance calibration, disarmed live compare, fuse LINEAR velocity only into the EKF behind a launch arg, then a LiDAR-vs-FCU guard (stop path → needs user OK). Hand-off: docs/reports/next_session_prompt.md.
