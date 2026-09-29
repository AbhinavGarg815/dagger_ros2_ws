#!/usr/bin/env python3
"""Unit tests for dagger_bringup/person_tracking.py (no ROS, no rover).

    python3 tools/test_person_tracking.py

Frame-by-frame synthetic detections. Checks the rules the follow path relies on:
IDs are stable while a person is seen, never reused, the selection never moves
to another person on its own, and a dropped selection is reported LOST.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "dagger_bringup"))
from dagger_bringup.person_tracking import (COASTING, LOST, NONE, TRACKING,  # noqa: E402
                                            Tracker, iou_matrix)

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail and not ok else ""))


def box(cx, cy=240, w=60, h=160):
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


# 1. IoU sanity
m = iou_matrix([box(100)], [box(100), box(130), box(400)])
check("iou: identical 1, overlapping in (0,1), disjoint 0",
      abs(m[0, 0] - 1) < 1e-9 and 0 < m[0, 1] < 1 and m[0, 2] == 0, str(m))

# 2. two people walking in parallel keep their IDs
t = Tracker()
for k in range(30):
    tr = t.update([box(100 + 3 * k), box(400 - 3 * k)], [0.9, 0.9])
ids = sorted(x.id for x in tr)
check("two walkers keep IDs 1 and 2 for 30 frames", ids == [1, 2] and t.next_id == 3, str(ids))

# 3. selection + short occlusion (coasting) + re-appearance keeps the ID
t = Tracker(max_missed=5)
for k in range(5):
    t.update([box(200 + 5 * k)], [0.9])
check("select existing track", t.select(1) and t.state() == TRACKING)
for k in range(3):                       # hidden for 3 frames
    t.update([], [])
check("hidden 3 frames -> COASTING, still selected", t.state() == COASTING and t.selected_id == 1)
t.update([box(240)], [0.9])              # reappears where constant velocity predicts
check("re-appears on the predicted track -> same ID, TRACKING",
      t.state() == TRACKING and [x.id for x in t.tracks] == [1], str([x.id for x in t.tracks]))

# 4. selected person dropped -> LOST; a newcomer at the same place is NOT the target
for k in range(7):
    t.update([], [])
check("hidden > max_missed -> LOST, id kept", t.state() == LOST and t.selected_id == 1)
t.update([box(240)], [0.9])
check("newcomer gets a NEW id, selection stays LOST",
      [x.id for x in t.tracks] == [2] and t.state() == LOST and t.selected_id == 1)

# 5. selecting a non-existent id changes nothing; clear works
check("select unknown id refused", t.select(99) is False and t.selected_id == 1)
t.clear()
check("clear -> NONE", t.state() == NONE and t.selected_id == -1)

# 6. IDs are never reused across many births and deaths
t = Tracker(max_missed=0)
seen = []
for k in range(20):
    tr = t.update([box(100 + 20 * (k % 5))] if k % 2 == 0 else [], [0.9] if k % 2 == 0 else [])
    seen += [x.id for x in tr]
check("ids never reused", len(set(seen)) == len(seen), str(seen))

# 7. weak detections do not start tracks but do extend them
t = Tracker(new_track_score=0.5)
t.update([box(300)], [0.3])
check("weak detection starts no track", t.tracks == [])
t.update([box(300)], [0.9])
t.update([box(302)], [0.3])
check("weak detection extends an existing track",
      len(t.tracks) == 1 and t.tracks[0].visible and t.tracks[0].missed == 0)

print(f"\n{sum(results)}/{len(results)} passed")
sys.exit(0 if all(results) else 1)
