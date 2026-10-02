# dagger_ros2_ws

ROS 2 Humble workspace of the Dagger outdoor tracked rover (Jetson Orin Nano, ArduPilot via
MAVROS, RS-Helios-16P LiDAR, RealSense D435): navigation, SLAM, and person / ArUco-marker following.

- **[COMMANDS.md](COMMANDS.md)** — how to build, bring up, drive, record, test and debug it.
- **[docs/](docs/README.md)** — the engineering log (`docs/reports/issues_and_fixes.md`: every issue,
  cause, fix and live-test result), the ArduPilot checklist, the next-session hand-off, and the
  learnings kept by the coding agent (`docs/learnings/`).
- `src/dagger_bringup` — nodes, launch files, parameters, maps, the follow web page.
- `src/dagger_msgs`, `src/dagger_description` — messages and the robot model.
- `tools/` — operator scripts, analysis tools and offline tests.
- `dagger.repos` — third-party packages at their pinned commits.
