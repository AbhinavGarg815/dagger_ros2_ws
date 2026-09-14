#!/usr/bin/env bash
# LAPTOP E-STOP (TX is dead). Double coverage:
#   1) FCU -> HOLD (ArduPilot stops regardless of what ROS sends)
#   2) /estop true streamed at 10 Hz -> twist_mux lock + cmd_vel_safety zero output
# Keep it running; Ctrl-C then `bash tools/estop_clear.sh` to release.
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: HOLD}" &
exec ros2 topic pub -r 10 /estop std_msgs/msg/Bool "{data: true}"
