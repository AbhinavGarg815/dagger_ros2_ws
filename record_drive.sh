#!/usr/bin/env bash
set +u

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

BAG_DIR="$HOME/dagger_bags"
mkdir -p "$BAG_DIR"
OUT="$BAG_DIR/dagger_drive_$(date +%Y%m%d_%H%M%S)"

STORAGE_OPT=""
if dpkg -s ros-humble-rosbag2-storage-mcap >/dev/null 2>&1; then
  STORAGE_OPT="-s mcap"
fi

echo "=== free disk on \$HOME ==="
df -h "$HOME" | tail -1
echo "=== recording to: $OUT (${STORAGE_OPT:-default sqlite3}) ==="
echo "Drive now. Press Ctrl-C once to stop and finalize the bag."
echo ""

exec ros2 bag record $STORAGE_OPT -o "$OUT" \
  /rslidar_points /rslidar_imu_data \
  /mavros/imu/data /mavros/imu/data_raw /mavros/imu/mag \
  /mavros/global_position/raw/fix /mavros/global_position/global \
  /mavros/global_position/local /mavros/global_position/raw/gps_vel \
  /mavros/global_position/compass_hdg /mavros/global_position/rel_alt \
  /mavros/local_position/odom /mavros/local_position/pose \
  /mavros/wheel_odometry/odom /mavros/altitude \
  /mavros/state /mavros/setpoint_velocity/cmd_vel_unstamped \
  /mavros/rc/in /mavros/rc/out \
  /tf /tf_static
