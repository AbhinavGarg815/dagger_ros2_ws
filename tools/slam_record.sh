#!/usr/bin/env bash
# Record a mapping drive so it can be re-mapped OFFLINE with different settings.
#   bash tools/slam_record.sh <name>            # 2D: scans, odom, TF (~small)
#   bash tools/slam_record.sh <name> clouds     # + raw /rslidar_points for 3D SLAM work
#                                                 (~8 MB/s: 5 min is ~2.4 GB)
# -> ~/dagger_bags/slam_<name>_<HHMMSS>. Ctrl-C to stop.
set -e
name=${1:?usage: slam_record.sh <name> [clouds]}
# Timestamped: ros2 bag refuses an existing folder, and on 2026-09-26 a second
# 'lane2' start was refused that way -- the drive that made the better map was
# never recorded. The same trap was fixed in follow_solo.sh the day before.
out=~/dagger_bags/slam_${name}_$(date +%H%M%S)
topics="/scan /scan_band /scan_safety /odometry/filtered /mavros/imu/data_raw \
  /mavros/local_position/odom /mavros/gpsstatus/gps1/raw /mavros/global_position/global \
  /gps/heading_imu /tf /tf_static /slam/map /slam/pose \
  /slam/match_score /slam/localized /slam/gnss_pose /initialpose /mavros/rc/in \
  /lidar_odom/odometry /lidar_odom/status"
[ "${2:-}" = "clouds" ] && topics="$topics /rslidar_points /patchworkpp/nonground"
mkdir -p ~/dagger_bags
echo "recording -> $out   (Ctrl-C to stop)"
exec ros2 bag record -s mcap -o "$out" $topics
