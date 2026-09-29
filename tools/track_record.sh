#!/usr/bin/env bash
# Record what the person tracker sees, for offline replay and tuning.
#   bash tools/track_record.sh <name>        -> ~/dagger_bags/track_<name>_<HHMMSS>
# Needs T1 (use_camera:=true), T2 and T4 up. NON-MOTION. Ctrl-C to stop.
# Contains the tracker's INPUTS (detections, scan, odometry, camera model, static
# TF) so tools/replay_tracker.py can re-run person_tracking.py on them with other
# settings, plus its outputs and the UI JPEGs (the recorder's subscription makes
# person_detector encode them) to check by eye what happened.
set -e
name=${1:?usage: track_record.sh <name>}
out=~/dagger_bags/track_${name}_$(date +%H%M%S)
mkdir -p ~/dagger_bags
echo "recording -> $out   (Ctrl-C to stop)"
exec ros2 bag record -s mcap -o "$out" \
  /follow/detections /follow/person_detections /follow/search_hint /scan_safety /odometry/filtered /tf_static /d435/color/camera_info \
  /follow/tracks /follow/select /follow/state /follow/target /follow/track_id \
  /follow/target_source /follow/ui/image/compressed
