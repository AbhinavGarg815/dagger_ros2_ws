#!/usr/bin/env bash
# Fetch and build Patchwork++ ground segmentation at the pinned commit.
#   bash tools/fetch_patchworkpp.sh
# src/patchwork-plusplus/ is gitignored like the other vendored packages; this
# script is how it is reproduced. Only the ROS 2 wrapper (ros/) is built: it
# compiles the core from ../cpp itself, so colcon must ignore cpp/ and python/.
# Single worker: parallel C++ builds have OOM'd this Orin before (A1).
set -euo pipefail
PIN=3e6903a
cd "$(dirname "$0")/.."
if [ ! -d src/patchwork-plusplus ]; then
  git clone -q https://github.com/url-kaist/patchwork-plusplus.git src/patchwork-plusplus
fi
git -C src/patchwork-plusplus fetch -q origin
git -C src/patchwork-plusplus checkout -q "$PIN"
touch src/patchwork-plusplus/cpp/COLCON_IGNORE src/patchwork-plusplus/python/COLCON_IGNORE
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 colcon build --packages-select patchworkpp --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_SYSTEM_EIGEN3=ON
