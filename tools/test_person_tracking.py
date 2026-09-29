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
                                            TRACKING, Detection, iou_matrix,
                                            lidar_cluster, range_at)
from dagger_bringup.person_tracking import Tracker as NewTracker  # noqa: E402
from dagger_bringup.person_tracking import SEARCHING, person_sized  # noqa: E402


def Tracker(**kw):
    """Tests below were written for immediate ids and 1 s survival; the ID-stability
    features (confirmation, 2 s survival) are tested separately at the end."""
    kw.setdefault("min_hits", 1)
    kw.setdefault("max_missed", 10)
    kw.setdefault("select_requires_range", False)
    kw.setdefault("isolated_hold_timeout", kw.get("lidar_only_timeout", 5.0))
    return NewTracker(**kw)

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
wx = np.full(200, 6.0); wy = np.linspace(-3, 3, 200)           # a wall 6 m ahead
check("range_at refuses a slice of a wall (surface continues on both sides)",
      range_at(wx, wy, (0.29, 0.0), 0.0, math.radians(3)) is None)
px2, py2 = circle_scan([(3.0, 0.0)])
px2, py2 = np.concatenate([px2, wx]), np.concatenate([py2, wy])
xy2 = range_at(px2, py2, (0.29, 0.0), 0.0, math.radians(5))
check("... but a person standing in front of that wall is still ranged",
      xy2 is not None and abs(xy2[0] - 2.8) < 0.2, str(xy2))
got, why = lidar_cluster(px, py, (3.0, 0.0), 0.8)
check("lidar_cluster accepts a person-sized cluster", got is not None and why is None)

# ---------------- ID stability (defaults of the real Tracker) ----------------
t = NewTracker()
t.update([det_at(3.0, 0.2)], 0, T0)
t.update([det_at(3.0, 0.2)], DT, T0)
check("new person is tentative (no id, not shown) before min_hits", t.confirmed() == [] and not t.select(1))
tr, _ = t.update([det_at(3.0, 0.2)], 2 * DT, T0)
check("confirmed on the 3rd sighting -> id 1, selectable", [x.id for x in tr] == [1] and t.select(1))
t = NewTracker()
t.update([Detection(box(300), .9)], 0)
t.update([], DT)
t.update([], 2 * DT)
t.update([], 3 * DT)
check("a one-frame false detection never gets an id", t.next_id == 1 and t.tracks == [])

# a LiDAR range glitch (the range jumps 2 m for one frame) must not change the id
t = NewTracker()
for k in range(5):
    t.update([det_at(3.0, 0.2)], k * DT, T0)
t.select(1)
glitch = det_at(3.0, 0.2)
glitch.xy = (5.0, 0.2)                                   # range_at picked the wall behind
tr, tgt = t.update([glitch], 5 * DT, T0)
check("one-frame range glitch: same id, no new track, no target from the bad range",
      [x.id for x in tr] == [1] and len(t.tracks) == 1 and tgt is None, f"ids {[x.id for x in tr]}")
tr, tgt = t.update([det_at(3.0, 0.2)], 6 * DT, T0)
check("... and the next good frame targets again at the true range",
      tgt is not None and abs(tgt[0] - 3.0) < 1e-6)
# the person really is somewhere else consistently -> re-anchor after dispute_frames
for k in range(7, 10):
    moved = det_at(3.0, 0.2)
    moved.xy = (4.2, 0.2)
    tr, tgt = t.update([moved], k * DT, T0)
check("range consistently different for 3 frames -> re-anchored, same id",
      [x.id for x in tr] == [1] and abs(t.selected().pos[0] - 4.2) < 1e-6)

# weak (side-on) detections keep a track alive but never start one
t = NewTracker()
for k in range(4):
    t.update([det_at(3.0, 0.2)], k * DT, T0)
for k in range(4, 34):                                   # 3 s of weak detections
    tr, _ = t.update([det_at(3.0, 0.2, score=0.3)], k * DT, T0)
check("3 s of weak (0.3) detections keep id 1 alive", [x.id for x in tr] == [1] and tr[0].visible)
t = NewTracker()
for k in range(5):
    t.update([det_at(3.0, 0.2, score=0.3)], k * DT, T0)
check("weak detections alone never start a track", t.tracks == [])

# a duplicate detection on top of a tracked person does not become a second person
t = NewTracker()
for k in range(4):
    t.update([det_at(3.0, 0.2)], k * DT, T0)
dup = Detection(det_at(3.0, 0.2).box + np.array([5, 5, 5, 5]), 0.8, 0.0, None)
for k in range(4, 8):
    t.update([det_at(3.0, 0.2), dup], k * DT, T0)
check("duplicate box over a tracked person -> no second track", len(t.tracks) == 1)

# survival: 1.5 s without any detection keeps the id (max_missed 20 = 2 s)
t = NewTracker()
for k in range(4):
    t.update([det_at(3.0, 0.2)], k * DT, T0)
for k in range(4, 19):
    t.update([], k * DT, T0)
tr, _ = t.update([det_at(3.0, 0.25)], 19 * DT, T0)
check("1.5 s dropout -> same id when seen again", [x.id for x in tr] == [1])

# a person partly behind another gets the FRONT person's LiDAR range (range_at takes
# the nearest cluster): that must never move them onto the front person
t = NewTracker()
for k in range(5):
    t.update([det_at(4.8, 0.4), det_at(3.0, 0.0)], k * DT, T0)
sel = next(x.id for x in t.confirmed() if x.pos[0] > 4)
t.select(sel)
stolen = False
for k in range(5, 15):
    a_ = det_at(4.8, 0.3)
    a_.xy = (3.0, 0.0)                                   # A's box, but B's cluster
    tr, tgt = t.update([a_, det_at(3.0, 0.0)], k * DT, T0)
    if tgt is not None and math.hypot(tgt[0] - 3.0, tgt[1]) < 0.5:
        stolen = True
check("range borrowed from the person in front: A never re-anchored onto B, no target at B",
      not stolen and t.selected() is not None and t.selected().pos[0] > 4.0,
      f"A pos {t.selected().pos if t.selected() else None}")
d1, d2 = det_at(4.8, 0.3), det_at(3.0, 0.0)
d1.xy = (3.02, 0.01)
NewTracker().update([d1, d2], 0, T0)
check("same cluster for two boxes -> the smaller (farther) box loses its range",
      d1.xy is None and d2.xy is not None)

# ---------------- round 2: true positives only, never lose, re-identify ----------------
rng_ = np.random.default_rng(7)


def sig(base=None, noise=0.0):
    v = rng_.normal(size=512) if base is None else base + rng_.normal(size=512) * noise
    return v / np.linalg.norm(v)


def with_emb(d, e):
    d.emb = e
    return d


# a one-off confident detection + weak ones (the orange sign) never becomes a person
t = NewTracker()
t.update([det_at(4.0, 0.5, score=0.6, ranged=False)], 0, T0)
for k in range(1, 20):
    t.update([det_at(4.0, 0.5, score=0.3, ranged=False)], k * DT, T0)
check("1 confident + weak detections never confirm (no id)", t.confirmed() == [])
t = NewTracker()
for k in range(5):
    t.update([det_at(4.0, 0.5, ranged=False)], k * DT, T0)
check("confirmed but never LiDAR-ranged -> cannot be selected",
      [x.id for x in t.confirmed()] == [1] and not t.select(1))

FY = 606.0
box_p = det_at(3.0, 0.0).box                      # a 1.7 m person at 3 m
check("person_sized: 1.7 m person at 3 m -> yes", person_sized(box_p, 3.0, FX, FY, 480 * 4))
check("person_sized: same box ranged at 8 m (wall behind) -> no", not person_sized(box_p, 8.0, FX, FY, 480 * 4))
sign = (300, 200, 360, 260)                       # 0.3 m tall at 3 m
check("person_sized: a 0.3 m tall object -> no", not person_sized(sign, 3.0, FX, FY, 480))
cut = (250, 0, 390, 480)                          # fills the frame (cut top+bottom) at 1.2 m
check("person_sized: cut-off box -> only 'too tall' can reject", person_sized(cut, 1.2, FX, FY, 480))

# too close for the LiDAR but in the camera: stays the target at near_range
t = NewTracker()
for k in range(5):
    t.update([det_at(2.0, 0.1)], k * DT, T0)
t.select(1)
nd = det_at(1.2, 0.1, ranged=False)
nd.near = True
tr, tgt = t.update([nd], 5 * DT, T0)
check("too close for the LiDAR (near) -> target at near_range, source 'near', same id",
      tgt is not None and tgt[2] == "near" and abs(math.hypot(tgt[0], tgt[1]) - 1.0) < 1e-6
      and t.state() == TRACKING)

# isolated LiDAR hold outlives lidar_only_timeout; a crowded one does not
def hold_run(extra_pts):
    t = NewTracker(lidar_only_timeout=2.0, isolated_hold_timeout=4.0)
    for k in range(5):
        t.update([det_at(3.0, 0.3)], k * DT, T0)
    t.select(1)
    lost = None
    for k in range(5, 80):
        f = min(k - 5, 10) / 10.0
        x, y = 3.0 - 1.0 * f, 0.3 + 2.7 * f           # walks out of view, then stands
        vis = abs(math.atan2(y, x)) < math.radians(27)
        px, py = circle_scan([(x, y)] + extra_pts)
        t.update([det_at(x, y)] if vis else [], k * DT, T0, (px, py))
        if lost is None and t.state() in (LOST, SEARCHING):
            lost = k * DT
    return lost
iso, crowd = hold_run([]), hold_run([(2.0, 3.8)])      # a post 0.8 m from where they stand
check("isolated person out of view: held past lidar_only_timeout, until isolated_hold_timeout",
      iso is not None and iso > 4.0, f"lost at {iso}")
check("something within 1 m: the hold ends at lidar_only_timeout", crowd is not None and crowd < 3.2,
      f"lost at {crowd}")

# re-identification of the selected person
eA, eB = sig(), sig()


def reid_scene(newcomers, window=30.0, lost_s=1.0):
    t = NewTracker(reid_window=window)
    for k in range(15):                                # A tracked 1.5 s, gallery learns
        t.update([with_emb(det_at(3.0, 0.2), sig(eA, 0.02))], k * DT, T0)
    t.select(1)
    k = 15
    while t.state() not in (LOST, SEARCHING) and k < 100:   # A vanishes (walked away fast)
        t.update([], k * DT, T0)
        k += 1
    t_lost = k * DT
    k += int(lost_s / DT)
    out = []
    for n in range(12):
        dl = [with_emb(det_at(x, y), sig(e, 0.03)) for (x, y, e) in newcomers]
        tr, tgt = t.update(dl, k * DT, T0)
        out.append((t.state(), t.selected_id, tgt))
        k += 1
    return t, out, t_lost

t, out, _ = reid_scene([(4.0, -0.5, eA)])
check("A returns (new track, same looks) -> re-identified: same id 1, TRACKING, target again",
      out[-1][0] == TRACKING and out[-1][1] == 1 and out[-1][2] is not None
      and any(x.id == 1 for x in t.confirmed()), str(out[-1][:2]))
t, out, _ = reid_scene([(4.0, -0.5, eB)])
check("a different-looking person is never re-identified as A (stays SEARCHING)",
      all(o[0] == SEARCHING and o[2] is None for o in out))
t, out, _ = reid_scene([(4.0, -0.5, eA), (4.5, 1.0, eA)])   # equally similar to A
check("two equally good look-alikes, no clear margin -> no re-identification",
      all(o[2] is None for o in out))
t, out, _ = reid_scene([(4.0, -0.5, eA), (4.5, 1.0, sig(eA, 0.03))])
check("A and a weaker look-alike -> A (the clear best) is re-identified",
      out[-1][1] == 1 and out[-1][2] is not None and abs(out[-1][2][0] - 4.0) < 0.3)
t, out, _ = reid_scene([(4.0, -0.5, eA)], window=0.5, lost_s=1.0)
check("after the re-identification window -> LOST for good, no re-link",
      all(o[0] == LOST and o[2] is None for o in out))
# someone present at the loss is never the target, however similar
t = NewTracker()
for k in range(15):
    t.update([with_emb(det_at(3.0, 0.2), sig(eA, 0.02)),
              with_emb(det_at(5.0, -1.5), sig(eA, 0.03))], k * DT, T0)
t.select(1)
k = 15
while t.state() not in (LOST, SEARCHING) and k < 100:
    t.update([with_emb(det_at(5.0, -1.5), sig(eA, 0.03))], k * DT, T0)
    k += 1
for n in range(12):
    tr, tgt = t.update([with_emb(det_at(5.0, -1.5), sig(eA, 0.03))], (k + n) * DT, T0)
check("a look-alike who was there when A was lost is never re-identified as A",
      t.state() == SEARCHING and tgt is None)

# appearance veto: an unranged box that looks nothing like the track is not them
t = NewTracker()
for k in range(10):
    t.update([with_emb(det_at(3.0, 0.2, ranged=False), sig(eA, 0.02))], k * DT, T0)
tr, _ = t.update([with_emb(det_at(3.0, 0.2, ranged=False), sig(eB, 0.02))], 10 * DT, T0)
check("box-only match vetoed when appearance clearly differs", not tr[0].visible)

# search hint: where the lost person probably is
t = NewTracker()
for k in range(15):
    t.update([with_emb(det_at(3.0, 0.2 + 0.05 * k), sig(eA, 0.02))], k * DT, T0)
t.select(1)
k = 15
while t.state() not in (LOST, SEARCHING):
    t.update([], k * DT, T0)
    k += 1
h = t.search_hint(k * DT)
check("search hint points where the lost person was heading", h is not None and h[1] > 0.9,
      str(h))

print(f"\n{sum(results)}/{len(results)} passed")
sys.exit(0 if all(results) else 1)
