#!/usr/bin/env bash
# FIRST DRIVEN FOLLOW-ME TEST, one person. THE USER RUNS THIS, then walks to the rover.
#
#   bash tools/follow_solo.sh <bag_name> HH:MM:SS [HH:MM:SS ...]
#   e.g. bash tools/follow_solo.sh follow_walk_1 14:05:00 14:07:00
#
# At each clock slot it polls tools/follow_solo_gate.py for POLL_S seconds (60 by
# default), printing each verdict, so you can step back or forward until it
# passes instead of having to hit the window on the first try.
# The gate passes only when
# the rover is armed in GUIDED AND you are standing 2.7-3.3 m in front of it --
# inside the controller's deadband, so the moment follow is enabled it commands
# ZERO and only starts moving when you walk away. No slot passes, nothing is
# enabled and nothing moves.
#
# On PASS: start the bag and the status log, enable follow, run for DURATION_S,
# then disable and stop recording. Disabling also happens on Ctrl-C or any exit.
#
# KEEP THE RC TRANSMITTER IN YOUR HAND. This is the first time this rover drives
# itself at a person; the collision monitor stops it ~1.45 m out and the FCU
# brakes in 0.14 m at 0.4 m/s, but the RC is the thing that does not depend on
# any of the software under test.
set -u
name=$1; shift
DURATION_S=${DURATION_S:-60}
POLL_S=${POLL_S:-60}
cd "$(dirname "$0")/.."

disable_follow() {
  echo "$(date +%H:%M:%S) disabling follow"
  timeout 10 ros2 topic pub --once /follow/enable std_msgs/Bool "{data: false}" >/dev/null 2>&1
}
trap 'disable_follow; kill -INT ${rec:-0} ${stat:-0} 2>/dev/null; exit 130' INT TERM

for slot in "$@"; do
  while [ "$(date +%H:%M:%S)" \< "$slot" ]; do sleep 0.2; done
  # Retry for POLL_S rather than judging one instant: the operator is standing in
  # front of the rover and cannot see this terminal, so the gate has to let them
  # walk themselves into the window. Every safety condition is unchanged -- this
  # only decides WHEN they are checked.
  g=""
  poll_end=$(( $(date +%s) + POLL_S ))
  while [ "$(date +%s)" -lt "$poll_end" ]; do
    g=$(python3 tools/follow_solo_gate.py 2>/dev/null | grep GATE)
    echo "  $g"
    echo "$g" | grep -q PASS && break
    sleep 2
  done
  echo "slot $slot: $g"
  if echo "$g" | grep -q PASS; then
    bash tools/pf_record.sh "$name" > "/tmp/follow_rec_$name.log" 2>&1 &
    rec=$!
    python3 tools/follow_status.py -r 2 > "$HOME/dagger_bags/${name}_status.log" 2>&1 &
    stat=$!
    sleep 2
    echo "$(date +%H:%M:%S) ENABLING FOLLOW for ${DURATION_S}s -- walk away slowly"
    timeout 10 ros2 topic pub --once /follow/enable std_msgs/Bool "{data: true}" >/dev/null
    sleep "$DURATION_S"
    disable_follow
    sleep 3
    kill -INT "$stat" 2>/dev/null
    kill -INT "$rec" 2>/dev/null
    echo "$(date +%H:%M:%S) done -- status log: ~/dagger_bags/${name}_status.log"
    exit 0
  fi
done
echo "$(date +%H:%M:%S) no slot passed the gate -- follow was never enabled, nothing moved"
