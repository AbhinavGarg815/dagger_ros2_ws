#!/usr/bin/env bash
# Record a small path-following test bag (no point clouds) for tools/pf_analyze.py.
#   bash tools/pf_record.sh [name]      -> ~/dagger_bags/pf_<name|timestamp>
# Start BEFORE sending the goal, Ctrl-C after the rover stops.
set -e
name=${1:-$(date +%Y%m%d_%H%M%S)}
out=~/dagger_bags/pf_${name}
mkdir -p ~/dagger_bags
echo "recording -> $out   (Ctrl-C to stop)"
exec ros2 bag record -s mcap -o "$out" \
  /plan /odometry/filtered /cmd_vel_nav /mavros/setpoint_velocity/cmd_vel_unstamped \
  /mavros/imu/data /mavros/imu/data_raw /mavros/gpsstatus/gps1/raw /mavros/state \
  /mavros/rc/out /mavros/local_position/velocity_body /lookahead_point \
  /tf /tf_static /rosout /navigate_to_pose/_action/status
