#!/usr/bin/env bash
# Runs one Dagger command with the environment the stack uses in a terminal
# (~nvidia/.bashrc): ROS 2 Humble + this workspace, domain 0, Fast-DDS with the
# 64 MB shared-memory profile. Every dagger-*.service starts through this, as
# user nvidia, so the services and a logged-in engineer share DDS natively.
set -e
source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/ros2_ws/install/dagger_bringup/share/dagger_bringup/config/fastdds_dagger.xml
export PYTHONUNBUFFERED=1          # log lines reach journalctl as they happen
exec "$@"
