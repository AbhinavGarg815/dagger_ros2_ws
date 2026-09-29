#!/usr/bin/env python3
"""Unit tests for dagger_bringup/person_tracking.py (no ROS, no rover).

    python3 tools/test_person_tracking.py

Frame-by-frame synthetic people. Checks the rules the follow path relies on:
IDs stable and never reused; the selection never moves to another person on its
own; people crossing in the image at different distances keep their IDs; the
selected person is held on the LiDAR out of view, re-confirmed by the camera,
refused on a wall-sized cluster, LOST after the hold times out; no range, no target.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "dagger_bringup"))
from dagger_bringup.person_tracking import (COASTING, HOLD_LIDAR, LOST, NONE,  # noqa: E402
                                            TRACKING, Detection, Tracker, iou_matrix,
                                            lidar_cluster, range_at)

FX, CX = 606.0, 320.0
DT = 0.1
T0 = (0.0, 0.0, 0.0)          # rover parked at the odom origin
results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail and not ok else ""))


def det_at(x, y, score=0.9, ranged=True, w_m=0.5):
    """A person at base (x, y): box from a simple pinhole looking along +x."""
    u = CX - FX * y / x
    half = FX * (w_m / 2) / x
    h = FX * 1.7 / x
    box = (u - half, 240 - h / 2, u + half, 240 + h / 2)
    return Detection(box, score, math.atan2(y, x), (x, y) if ranged else None)


def circle_scan(people, r=0.25, step=0.03):
    """LiDAR points on the front arc of each person (base frame)."""
    px, py = [], []
    for x, y in people:
        a0 = math.atan2(-y, -x)                  # face the rover
        for a in np.arange(a0 - 1.2, a0 + 1.2, step * 4):
            px.append(x + r * math.cos(a))
            py.append(y + r * math.sin(a))
    return np.array(px), np.array(py)


def box(cx, cy=240, w=60, h=160):
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


# ---------------- phase 1 rules, image-only (no odometry) ----------------
m = iou_matrix([box(100)], [box(100), box(130), box(400)])
check("iou: identical 1, overlapping in (0,1), disjoint 0",
      abs(m[0, 0] - 1) < 1e-9 and 0 < m[0, 1] < 1 and m[0, 2] == 0, str(m))

t = Tracker()
for k in range(30):
    tr, _ = t.update([Detection(box(100 + 3 * k), .9), Detection(box(400 - 3 * k), .9)], k * DT)
check("two walkers keep IDs 1 and 2 (image only)", sorted(x.id for x in tr) == [1, 2] and t.next_id == 3)

t = Tracker(max_missed=5)
for k in range(5):
    t.update([Detection(box(200 + 5 * k), .9)], k * DT)
check("select existing track", t.select(1) and t.state() == TRACKING)
for k in range(3):
    t.update([], (5 + k) * DT)
check("hidden 3 frames -> COASTING, still selected", t.state() == COASTING and t.selected_id == 1)
t.update([Detection(box(240), .9)], 8 * DT)
check("re-appears on the predicted track -> same ID, TRACKING",
      t.state() == TRACKING and [x.id for x in t.tracks] == [1])
for k in range(7):
    t.update([], (9 + k) * DT)
check("hidden > max_missed -> LOST, id kept", t.state() == LOST and t.selected_id == 1)
t.update([Detection(box(240), .9)], 17 * DT)
check("newcomer gets a NEW id, selection stays LOST",
      [x.id for x in t.tracks] == [2] and t.state() == LOST and t.selected_id == 1)
check("select unknown id refused", t.select(99) is False and t.selected_id == 1)
t.clear()
check("clear -> NONE", t.state() == NONE and t.selected_id == -1)

t = Tracker(max_missed=0)
seen = []
for k in range(20):
    tr, _ = t.update([Detection(box(100 + 20 * (k % 5)), .9)] if k % 2 == 0 else [], k * DT)
    seen += [x.id for x in tr]
check("ids never reused", len(set(seen)) == len(seen))

t = Tracker(new_track_score=0.5)
t.update([Detection(box(300), .3)], 0)
check("weak detection starts no track", t.tracks == [])
t.update([Detection(box(300), .9)], DT)
t.update([Detection(box(302), .3)], 2 * DT)
check("weak detection extends an existing track", len(t.tracks) == 1 and t.tracks[0].visible)

# ---------------- phase 2: positions (odometry + LiDAR ranges) ----------------
# A (3 m) walks right->left while B (6 m) walks left->right: their boxes cross
# and overlap in the image. Image-only association swaps them; position must not.
t = Tracker()
idA = idB = None
swapped = False
for k in range(40):
    ya, yb = 1.2 - 0.06 * k, -1.2 + 0.06 * k           # both cross the centre line at k=20
    tr, _ = t.update([det_at(3.0, ya), det_at(6.0, yb)], k * DT, T0)
    if k == 0:
        idA = min(tr, key=lambda x: x.pos[0]).id
        idB = max(tr, key=lambda x: x.pos[0]).id
    a = next((x for x in tr if x.id == idA), None)
    if a is None or abs(a.pos[0] - 3.0) > 0.3:
        swapped = True
ov = iou_matrix([det_at(3.0, 0.0).box], [det_at(6.0, 0.0).box])[0, 0]
check("people crossing at 3 m and 6 m keep their IDs (position decides)",
      not swapped and len(t.tracks) == 2, f"boxes overlap IoU {ov:.2f} at the crossing")
tt = Tracker()
sw = False
for k in range(40):
    ya, yb = 1.2 - 0.06 * k, -1.2 + 0.06 * k
    d = [det_at(3.0, ya), det_at(6.0, yb)]
    for x in d:
        x.xy = None                                      # same scene, no LiDAR
    tr, _ = tt.update(d, k * DT)
    if k == 30:
        a = next(x for x in tr if x.id == 1)
        sw = a.box[0] > 320                              # track 1 (A, moving left) ended up right
check("(reference: image-only association CAN swap them)", True,
      f"image-only swapped: {sw}")

# target output: selected + ranged + inside [min_range, max_range]
t = Tracker(min_range=1.0, max_range=15.0)
t.update([det_at(3.0, 0.2)], 0, T0)
t.select(1)
_, tgt = t.update([det_at(3.0, 0.2)], DT, T0)
check("selected + ranged -> target from the camera", tgt is not None and tgt[2] == "camera"
      and abs(tgt[0] - 3.0) < 1e-6)
_, tgt = t.update([det_at(3.0, 0.2, ranged=False)], 2 * DT, T0)
check("no LiDAR range -> NO target (but still TRACKING)", tgt is None and t.state() == TRACKING)
t2 = Tracker(min_range=1.0, max_range=15.0)
t2.update([det_at(0.8, 0.0)], 0, T0)
t2.select(1)
_, tgt = t2.update([det_at(0.8, 0.0)], DT, T0)
check("closer than min_range -> no target", tgt is None)

# LiDAR hold: the selected person walks out of the camera view to the left
t = Tracker()
t.update([det_at(3.0, 0.3), det_at(4.0, -1.0)], 0, T0)
t.select(1)
path = [(3.0 - 0.1 * k, 0.3 + 0.25 * k) for k in range(1, 16)]   # ends at (1.5, 4.05): 70 deg left
held, ids_ok, bystander_ok = 0, True, True
for k, (x, y) in enumerate(path, start=1):
    in_view = abs(math.atan2(y, x)) < math.radians(27)
    dets = ([det_at(x, y)] if in_view else []) + [det_at(4.0, -1.0)]
    tr, tgt = t.update(dets, k * DT, T0, circle_scan([(x, y), (4.0, -1.0)]))
    if not in_view:
        held += tgt is not None and tgt[2] == "lidar"
    if tgt is not None and math.hypot(tgt[0] - x, tgt[1] - y) > 0.4:
        bystander_ok = False
    if t.selected() is None or t.selected().id != 1:
        ids_ok = False
check("selected person held on the LiDAR out of view (HOLD_LIDAR, source lidar)",
      held >= 8 and t.state() == HOLD_LIDAR, f"{held} held frames")
check("held target follows the person (never the bystander), id unchanged", bystander_ok and ids_ok)
# back into view: camera re-confirms the SAME id
back = [(1.5 + 0.15 * k, 4.05 - 0.4 * k) for k in range(1, 11)]
for k, (x, y) in enumerate(back, start=16):
    in_view = abs(math.atan2(y, x)) < math.radians(27)
    t.update(([det_at(x, y)] if in_view else []) + [det_at(4.0, -1.0)], k * DT, T0,
             circle_scan([(x, y), (4.0, -1.0)]))
check("back in view -> camera re-confirms the same id (TRACKING)",
      t.state() == TRACKING and t.selected().id == 1)

# hold refuses a wall-sized cluster
wall_x = np.linspace(0.5, 6.0, 120)
t = Tracker()
t.update([det_at(3.0, 0.3)], 0, T0)
t.select(1)
_, tgt = t.update([], DT, T0, (np.concatenate([wall_x, [3.0]]),
                                  np.concatenate([np.full(120, 0.35), [0.3]])))
check("LiDAR hold refuses a wall-merged cluster", tgt is None and "not a person" in (t.hold_refusal or ""),
      t.hold_refusal or "")

# hold times out -> LOST, never hands over to the bystander
t = Tracker(lidar_only_timeout=2.0)
t.update([det_at(3.0, 0.3), det_at(4.0, -1.0)], 0, T0)
t.select(1)
lost_at, stolen = None, False
for k in range(1, 60):
    f = min(k, 10) / 10.0                                 # walks out of view (left) in 1 s ...
    x, y = 3.0 - 1.0 * f, 0.3 + 2.7 * f                   # ... then stands at (2.0, 3.0)
    vis = abs(math.atan2(y, x)) < math.radians(27)
    tr, tgt = t.update(([det_at(x, y)] if vis else []) + [det_at(4.0, -1.0)], k * DT, T0,
                       circle_scan([(x, y), (4.0, -1.0)]))
    if tgt is not None and math.hypot(tgt[0] - 4.0, tgt[1] + 1.0) < 0.5:
        stolen = True
    if lost_at is None and t.state() == LOST:
        lost_at = k * DT
check("hold expires ~lidar_only_timeout after the last camera sighting -> LOST",
      lost_at is not None and 2.0 <= lost_at <= 3.5,
      f"lost at {lost_at}")
check("the hold never slides onto the bystander, before or after LOST", not stolen)

# the target walks BEHIND a bystander (LiDAR occluded): the hold must not take the bystander
t = Tracker()
t.update([det_at(3.0, 1.2), det_at(4.0, 0.0)], 0, T0)
t.select(1)
stolen, refusals = False, set()
for k in range(1, 25):
    x, y = 3.0 + 0.1 * min(k, 10), 1.2 - 0.12 * min(k, 10)          # walks to (4.0, 0.0)... behind B
    vis = k < 6
    people_scan = [(4.0, 0.0)] + ([(x, y)] if k < 6 else [])        # occluded by B from k=6
    tr, tgt = t.update(([det_at(x, y)] if vis else []) + [det_at(4.0, 0.0)], k * DT, T0,
                       circle_scan(people_scan))
    if tgt is not None and math.hypot(tgt[0] - 4.0, tgt[1]) < 0.5:
        stolen = True
    if t.hold_refusal:
        refusals.add(t.hold_refusal)
check("target occluded behind a bystander: hold refuses the bystander's cluster",
      not stolen and "nearest cluster is another tracked person" in refusals, str(refusals))

# geometry helpers keep target_selector's behaviour
px, py = circle_scan([(3.0, 0.0)])
xy = range_at(px, py, (0.29, 0.0), 0.0, math.radians(5))
check("range_at finds the person straight ahead", xy is not None and abs(xy[0] - 2.8) < 0.2, str(xy))
got, why = lidar_cluster(px, py, (3.0, 0.0), 0.8)
check("lidar_cluster accepts a person-sized cluster", got is not None and why is None)

print(f"\n{sum(results)}/{len(results)} passed")
sys.exit(0 if all(results) else 1)
