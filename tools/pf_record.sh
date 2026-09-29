#!/usr/bin/env bash
# Record a small path-following test bag (no point clouds) for tools/pf_analyze.py.
#   bash tools/pf_record.sh [name]      -> ~/dagger_bags/pf_<name>_<HHMMSS>
# Start BEFORE sending the goal, Ctrl-C after the rover stops.
set -e
# Timestamped even with a name: ros2 bag refuses an existing folder, which lost
# the lane2 SLAM drive (Q41) -- same trap here if a test name is reused.
name=${1:-run}
out=~/dagger_bags/pf_${name}_$(date +%H%M%S)
mkdir -p ~/dagger_bags
echo "recording -> $out   (Ctrl-C to stop)"
exec ros2 bag record -s mcap -o "$out" \
  /plan /odometry/filtered /cmd_vel_nav /cmd_vel_nav_raw /scan_safety /mavros/setpoint_velocity/cmd_vel_unstamped \
  /follow/target /follow/target_source /follow/track_id /follow/detections /follow/enable /cmd_vel_follow_raw /cmd_vel_follow \
  /mavros/imu/data /mavros/imu/data_raw /mavros/gpsstatus/gps1/raw /mavros/state \
  /mavros/rc/out /mavros/rc/in /mavros/local_position/velocity_body /lookahead_point \
  /tf /tf_static /rosout /navigate_to_pose/_action/status \
  /scan /slam/pose /slam/match_score /slam/localized /slam/gnss_pose \
  /mavros/global_position/global /gps/heading_imu
