#!/usr/bin/env bash
# Re-map a recorded drive OFFLINE with different slam_toolbox settings.
#   bash tools/slam_replay.sh <bag> <label> [param:=value ...]
#   bash tools/slam_replay.sh ~/dagger_bags/slam_lane3_164840 dist01 minimum_travel_distance:=0.1
# Runs in its own ROS domain (default 77) with sim time, so a live stack is not
# touched. Plays only /scan /tf /tf_static at 1x (async slam drops scans when it
# is slow, so replaying faster would not be faithful). Writes to
# ~/dagger_bags/replay/<label>/: poses.csv (every /slam/pose), map.{pgm,yaml},
# graph.{posegraph,data} (loadable for localization), cpu.txt. Compare runs with tools/slam_replay_eval.py.
set -e
bag=${1:?usage: slam_replay.sh <bag> <label> [param:=value ...]}
label=${2:?usage: slam_replay.sh <bag> <label> [param:=value ...]}
shift 2
export ROS_DOMAIN_ID=${REPLAY_DOMAIN:-77}
ws=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
out=~/dagger_bags/replay/$label
rm -rf "$out"; mkdir -p "$out"

overrides=()
for kv in "$@"; do overrides+=(-p "$kv"); done
# Same settings as slam.launch.py shadow mode, plus sim time.
/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node --ros-args -r __node:=slam_toolbox \
  --params-file "$ws/install/dagger_bringup/share/dagger_bringup/params/slam_toolbox.yaml" \
  -p mode:=mapping -p map_frame:=slam_map -p transform_publish_period:=0.0 -p use_sim_time:=true \
  "${overrides[@]}" \
  -r map:=/slam/map -r map_metadata:=/slam/map_metadata -r pose:=/slam/pose \
  > "$out/slam.log" 2>&1 &
slam_pid=$!
python3 "$ws/tools/slam_replay_eval.py" --log "$out/poses.csv" > "$out/logger.log" 2>&1 &
log_pid=$!
sleep 3

echo "[$label] domain $ROS_DOMAIN_ID, overrides: ${*:-none}; playing $bag"
ros2 bag play "$bag" --clock --topics /scan /tf /tf_static > "$out/play.log" 2>&1
sleep 3   # let the last scans be processed

ticks=$(getconf CLK_TCK)
read -r ut st < <(awk '{print $14, $15}' /proc/$slam_pid/stat)
el=$(awk -v s="$(awk '{print $22}' /proc/$slam_pid/stat)" -v t="$ticks" \
  '{print $1 - s/t}' /proc/uptime)
awk -v u="$ut" -v s="$st" -v t="$ticks" -v e="$el" \
  'BEGIN{printf "slam_toolbox cpu %.1f s over %.0f s wall = %.1f %%\n", (u+s)/t, e, 100*(u+s)/t/e}' | tee "$out/cpu.txt"

timeout 20 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$out/graph'}" > "$out/serialize.log" 2>&1 || true
timeout 20 ros2 run nav2_map_server map_saver_cli -t /slam/map -f "$out/map" \
  --ros-args -p map_subscribe_transient_local:=true -p use_sim_time:=true > "$out/saver.log" 2>&1 || true
kill -INT $log_pid $slam_pid 2>/dev/null || true
wait $log_pid $slam_pid 2>/dev/null || true
echo "[$label] done -> $out ($(($(wc -l < "$out/poses.csv") - 1)) poses)"
