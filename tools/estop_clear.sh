#!/usr/bin/env bash
# Release the laptop E-stop: publish /estop false (cmd_vel_safety latches until it sees it).
# Mode stays HOLD -- re-select GUIDED yourself before the next drive.
# Wait for every current /estop subscriber (twist_mux, cmd_vel_safety, the gateway):
# with -t alone, ros2 topic pub starts after the FIRST match, and on a busy Jetson the
# others can miss all five messages (2026-10-12: the gateway never saw the release).
n=$(ros2 topic info /estop 2>/dev/null | awk '/Subscription count/ {print $3}')
ros2 topic pub -w "${n:-1}" -t 10 -r 10 /estop std_msgs/msg/Bool "{data: false}"
