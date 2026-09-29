#!/usr/bin/env bash
# Test SLAM LOCALIZATION offline: play a recorded drive against a saved map.
#   bash tools/slam_loc_replay.sh <bag> <label> <map> [offset_x offset_y [param:=value ...]]
#   bash tools/slam_loc_replay.sh ~/dagger_bags/slam_lane1 l1 lane3fix
#   bash tools/slam_loc_replay.sh ~/dagger_bags/slam_lane1 l1_off2 lane3fix 2.0 0.0
# Same nodes as `slam.launch.py mode:=localization` (slam_toolbox localization +
# slam_geo_init seeding from GNSS), in ROS domain 77 with sim time. The bag's
# map->odom (gps_map_odom) is filtered out of /tf. offset_x/y shift the GNSS
# seed to test recovery from a bad start. Output: ~/dagger_bags/replay/<label>/
# poses.csv (SLAM), gnss.csv (GNSS in the map), geo.log, slam.log.
# Score: python3 tools/slam_replay_eval.py --loc <label> ...
set -e
bag=${1:?usage: slam_loc_replay.sh <bag> <label> <map> [offset_x offset_y]}
label=${2:?}; map=${3:?}
# overrides: param:=value -> slam_toolbox, geo.param:=value -> slam_geo_init
# (e.g. geo.max_speed:=1.0 when the recording starts moving before 10 still fixes)
extra=(); geo=()
for kv in "${@:6}"; do
  if [[ $kv == geo.* ]]; then geo+=(-p "${kv#geo.}"); else extra+=(-p "$kv"); fi
done
# always floats: the node declares them as double and rejects "0"
ox=$(printf %.3f "${4:-0}"); oy=$(printf %.3f "${5:-0}")
export ROS_DOMAIN_ID=${REPLAY_DOMAIN:-77}
ws=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
share=$ws/install/dagger_bringup/share/dagger_bringup
base=$map; [[ $map = /* ]] || base=$share/maps/$map
out=~/dagger_bags/replay/$label
rm -rf "$out"; mkdir -p "$out"

python3 "$ws/tools/tf_filter.py" > "$out/tf_filter.log" 2>&1 & pids=($!)
/opt/ros/humble/lib/slam_toolbox/localization_slam_toolbox_node --ros-args -r __node:=slam_toolbox \
  --params-file "$share/params/slam_toolbox.yaml" \
  -p mode:=localization -p map_frame:=map -p transform_publish_period:=0.05 \
  -p map_file_name:="$base" -p "map_start_pose:=[0.0, 0.0, 0.0]" -p use_sim_time:=true "${extra[@]}" \
  -r map:=/slam/map -r map_metadata:=/slam/map_metadata -r pose:=/slam/pose \
  > "$out/slam.log" 2>&1 & pids+=($!)
python3 "$ws/tools/slam_replay_eval.py" --log "$out/poses.csv" > "$out/logger.log" 2>&1 & pids+=($!)
"$ws/install/dagger_bringup/lib/dagger_bringup/slam_geo_init" --ros-args \
  -p geo_file:="${base}_geo.yaml" -p map_yaml:="${base}.yaml" -p use_sim_time:=true "${geo[@]}" -p offset_x:=$ox -p offset_y:=$oy \
  > "$out/geo.log" 2>&1 & pids+=($!)
sleep 5

echo "[$label] map $map, seed offset ($ox, $oy), overrides: ${*:6}; playing $bag"
ros2 bag play "$bag" --clock --remap /tf:=/tf_bag --topics /scan /tf /tf_static \
  /odometry/filtered /mavros/global_position/global /gps/heading_imu /mavros/gpsstatus/gps1/raw \
  > "$out/play.log" 2>&1
sleep 3
kill -INT "${pids[@]}" 2>/dev/null || true
wait "${pids[@]}" 2>/dev/null || true
grep -E "seeded|REJECTED|localized|LOST" "$out/geo.log" | sed 's/^/    /' || echo "    (never seeded -- see $out/geo.log)"
echo "[$label] done -> $out ($(($(wc -l < "$out/poses.csv") - 1)) SLAM poses)"
