#!/usr/bin/env bash
# Release the laptop E-stop: publish /estop false (cmd_vel_safety latches until it sees it).
# Mode stays HOLD -- re-select GUIDED yourself before the next drive.
ros2 topic pub -t 5 -r 10 /estop std_msgs/msg/Bool "{data: false}"
