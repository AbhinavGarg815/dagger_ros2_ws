#!/usr/bin/env bash
# Solo test: the USER runs this at the laptop, then walks to the rover with the RC.
# At each clock slot it runs tools/solo_gate.py; on PASS it starts the recorder, sends the
# goal (tools/send_goal.py args) and cancels it after MAX_S seconds. No slot passes -> nothing sent.
#   bash tools/solo_goal.sh <bag_name> "<send_goal args>" HH:MM:SS [HH:MM:SS ...]
#   e.g. bash tools/solo_goal.sh cm_step_in "8" 15:20:00 15:22:00
set -u
name=$1; goal_args=$2; shift 2
MAX_S=${MAX_S:-90}
cd "$(dirname "$0")/.."
python3 tools/solo_gate.py --save 2>/dev/null | grep -v INFO
for slot in "$@"; do
  while [ "$(date +%H:%M:%S)" \< "$slot" ]; do sleep 0.2; done
  g=$(python3 tools/solo_gate.py 2>/dev/null | grep GATE); echo "slot $slot: $g"
  if echo "$g" | grep -q PASS; then
    bash tools/pf_record.sh "$name" > "/tmp/solo_rec_$name.log" 2>&1 &
    rec=$!; sleep 2
    echo "$(date +%H:%M:%S) SENDING GOAL: send_goal.py $goal_args"
    # shellcheck disable=SC2086
    timeout -s INT "$MAX_S" python3 tools/send_goal.py $goal_args 2>&1 | grep -v "^\[INFO\]"
    echo "$(date +%H:%M:%S) goal finished/cancelled"
    sleep 5; kill -INT "$rec" 2>/dev/null
    exit 0
  fi
done
echo "$(date +%H:%M:%S) no slot passed the gate -- nothing sent"
