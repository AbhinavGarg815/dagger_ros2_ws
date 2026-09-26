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
# GATE_MIN_RANGE / GATE_MAX_RANGE / GATE_MAX_BEARING_DEG override where you may
# be standing when follow is switched on (default 2.0-6.0 m, within 20 deg).
# Those bound the START only -- following itself runs 1.0-8.0 m at any bearing
# inside the field of view.
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

# NEVER use `ros2 topic pub --once` here. It creates a publisher, fires
# immediately and tears down, so with a reliable subscriber the message is lost
# before discovery completes -- measured 2026-09-25: --once did not arrive at
# all, while -t 10 -r 5 arrived instantly. On the enable path a dropped message
# means the run silently does nothing; on the disable path it would mean the
# rover keeps following. Publish repeatedly, both directions.
set_follow() {
  timeout 15 ros2 topic pub -t 10 -r 5 /follow/enable std_msgs/Bool "{data: $1}" \
    >/dev/null 2>&1
}
disable_follow() {
  echo "$(date +%H:%M:%S) disabling follow"
  set_follow false
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
    # Unique per run: ros2 bag refuses to reuse an output folder, and on
    # 2026-09-25 a no-op earlier run had already taken plain "$name" -- so the
    # one run that actually drove recorded no bag at all, and the only record of
    # it was the status log. Losing the odometry of a first drive is not
    # recoverable, so a failed recorder now aborts BEFORE follow is enabled.
    stamp=$(date +%H%M%S)
    bag="${name}_${stamp}"
    bash tools/pf_record.sh "$bag" > "/tmp/follow_rec_$bag.log" 2>&1 &
    rec=$!
    python3 tools/follow_status.py -r 2 > "$HOME/dagger_bags/pf_${bag}_status.log" 2>&1 &
    stat=$!
    # Poll, do not sleep once: ros2 bag subscribes to 17 topics before it
    # creates the folder, which took longer than a flat 3 s wait and aborted two
    # runs whose recorders were in fact starting normally (and were then left
    # running as orphans). Give it 25 s, and check the process is still alive so
    # a genuine failure is caught immediately rather than waited out.
    ok=0
    for _ in $(seq 1 50); do
      if [ -d "$HOME/dagger_bags/pf_${bag}" ]; then ok=1; break; fi
      kill -0 "$rec" 2>/dev/null || break
      sleep 0.5
    done
    if [ "$ok" -ne 1 ]; then
      echo "RECORDER FAILED -- not enabling follow. See /tmp/follow_rec_$bag.log:"
      tail -3 "/tmp/follow_rec_$bag.log"
      kill -INT "$rec" 2>/dev/null; kill -INT "$stat" 2>/dev/null
      sleep 2; kill -KILL "$rec" 2>/dev/null
      exit 1
    fi
    # The folder existing is not enough. On 2026-09-25 (stop_test_173211) the
    # recorder took ~30 s to discover mavros's publishers: every /mavros/* topic
    # in the bag started six seconds AFTER the event under test, while
    # /odometry/filtered had been recording all along. Wait until the recorder
    # has actually subscribed to the FCU topics the analysis depends on.
    need="/mavros/rc/out /mavros/rc/in /mavros/local_position/velocity_body /mavros/setpoint_velocity/cmd_vel_unstamped"
    ok=0
    for _ in $(seq 1 90); do
      missing=0
      for tp in $need; do
        grep -q "Subscribed to topic '$tp'" "/tmp/follow_rec_$bag.log" 2>/dev/null || missing=1
      done
      [ "$missing" -eq 0 ] && { ok=1; break; }
      kill -0 "$rec" 2>/dev/null || break
      sleep 0.5
    done
    if [ "$ok" -ne 1 ]; then
      echo "RECORDER NOT SUBSCRIBED to the FCU topics after 45 s -- not enabling follow."
      grep -o "Subscribed to topic '[^']*'" "/tmp/follow_rec_$bag.log" | tail -5
      kill -INT "$rec" "$stat" 2>/dev/null; sleep 2; kill -KILL "$rec" 2>/dev/null
      exit 1
    fi
    # Subscribed is not receiving. Confirm the FCU streams are actually flowing,
    # then give the recorder a few seconds' head start before anything moves.
    # Exit code, not grep: "NOT FLOWING" contains "FLOWING".
    if ! python3 tools/fcu_flowing.py 3 2>/dev/null; then
      echo "FCU telemetry is not flowing -- run python3 tools/preflight.py. Not enabling follow."
      kill -INT "$rec" "$stat" 2>/dev/null; sleep 2; kill -KILL "$rec" 2>/dev/null
      exit 1
    fi
    sleep 3
    echo "$(date +%H:%M:%S) recording -> ~/dagger_bags/pf_${bag} (FCU telemetry flowing)"
    echo "$(date +%H:%M:%S) ENABLING FOLLOW for ${DURATION_S}s -- walk away slowly"
    set_follow true
    sleep "$DURATION_S"
    disable_follow
    sleep 3
    kill -INT "$stat" 2>/dev/null
    kill -INT "$rec" 2>/dev/null
    echo "$(date +%H:%M:%S) done -- bag ~/dagger_bags/pf_${bag}, status log ~/dagger_bags/pf_${bag}_status.log"
    exit 0
  fi
done
echo "$(date +%H:%M:%S) no slot passed the gate -- follow was never enabled, nothing moved"
