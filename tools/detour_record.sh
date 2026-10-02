#!/usr/bin/env bash
# Record an obstacle approach / detour test WITH raw clouds and both costmaps, so the
# low-obstacle pipeline can be replayed offline against the real approach (P14).
#   bash tools/detour_record.sh <name>     -> ~/dagger_bags/detour_<name>_<HHMMSS>
# ~7 MB/s (raw cloud): a 60 s run is ~0.4 GB. Start before the goal, Ctrl-C after the stop.
set -e
name=${1:?usage: detour_record.sh <name>}
# Timestamped: ros2 bag refuses an existing folder (lost the lane2 drive once, Q41).
out=~/dagger_bags/detour_${name}_$(date +%H%M%S)
mkdir -p ~/dagger_bags
echo "recording -> $out   (Ctrl-C to stop)"
exec ros2 bag record -s mcap -o "$out" \
  /rslidar_points /patchworkpp/nonground /scan /scan_band /scan_safety /scan_low \
  /plan /odometry/filtered /tf /tf_static /cmd_vel_nav /cmd_vel_nav_raw \
  /mavros/setpoint_velocity/cmd_vel_unstamped /global_costmap/costmap /local_costmap/costmap \
  /navigate_to_pose/_action/status /mavros/state
