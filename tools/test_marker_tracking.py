#!/usr/bin/env python3
"""Unit tests for dagger_bringup/marker_tracking.py (no ROS, no rover).

    python3 tools/test_marker_tracking.py

Geometry: ArUco markers are RENDERED into 640x480 frames through the rover's
real D435 model and upside-down mount (K and base->optical TF from the reid3
recording, 2026-09-30), detected with the same detector the node uses, and
located -- range, bearing and carrier heading against the known pose.
Tracking: synthetic sightings -- confirmation, allowed ids, loss -> search ->
resume, motion gate, detector stall, one target at a time.
"""
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "dagger_bringup"))
from dagger_bringup.marker_tracking import MarkerObs, MarkerTracker, aruco_detector  # noqa: E402
from dagger_bringup.person_tracking import LOST, NONE, SEARCHING, TRACKING  # noqa: E402

from marker_sim import CAM_R, CAM_T, DIST, K, render  # noqa: E402

results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail and not ok else ""))


detect = aruco_detector("DICT_4X4_50")


def locate(frame, size=0.15, pts=None, trk=None):
    trk = trk or MarkerTracker()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return [trk.observe(mid, c, K, DIST, size, CAM_T, CAM_R, pts) for mid, c in detect(gray)]


def bearing_err(o, x, y):
    return abs(math.degrees(math.atan2(math.sin(o.bearing - math.atan2(y, x)),
                                       math.cos(o.bearing - math.atan2(y, x)))))


print("geometry (rendered through the rover's real camera + mount):")
worst = {"near": 0.0, "far": 0.0}
worst_b = 0.0
for band, x, y in [("near", 1.2, 0.0), ("near", 2.0, 0.0), ("near", 3.0, 0.8), ("near", 4.0, -1.2),
                   ("near", 5.0, 1.0), ("far", 6.0, 0.0), ("far", 6.0, 2.0)]:
    obs = locate(render([(7, 0.15, (x, y, 1.0), 180.0)]))
    ok = [o for o in obs if o.marker_id == 7 and o.reject is None and o.xy is not None]
    if not ok:
        worst[band] = 99.0
        print(f"    marker at ({x}, {y}) not located: {[o.reject for o in obs]}")
        continue
    o = ok[0]
    r_true = math.hypot(x, y)
    worst[band] = max(worst[band], abs(math.hypot(*o.xy) - r_true) / r_true)
    worst_b = max(worst_b, bearing_err(o, x, y))
check("15 cm marker at 1.2-5.1 m (>= 19 px): range within 2.5 %", worst["near"] < 0.025,
      f"worst {100 * worst['near']:.1f} %")
check("... at 6-6.3 m (~15 px): range within 6 %", worst["far"] < 0.06,
      f"worst {100 * worst['far']:.1f} %")
check("... bearing within 1 deg everywhere", worst_b < 1.0, f"worst {worst_b:.2f} deg")

o = locate(render([(7, 0.15, (2.0, 0.3, 1.0), 180.0 + 30.0)]))[0]
check("carrier turned 30 deg left, 2 m (marker on its back): heading +30 +-5 deg",
      o.reject is None and abs(math.degrees(o.heading) - 30.0) < 5.0,
      f"{math.degrees(o.heading):.1f} deg, reject {o.reject}")
# a flat square seen small and nearly head-on has two equally good poses: the
# heading must then be UNKNOWN (NaN), never a confident wrong angle
wrong, known, total = [], 0, 0
for dist in (2.0, 3.0, 4.0):
    for turn in (0.0, 10.0, 20.0, 45.0):
        for seed in range(3):
            for o in locate(render([(7, 0.15, (dist, 0.3, 1.0), 180.0 + turn)], seed)):
                if o.reject is None:
                    total += 1
                    if o.heading == o.heading:
                        known += 1
                        if abs(math.degrees(o.heading) - turn) > 8.0:
                            wrong.append((dist, turn, round(math.degrees(o.heading))))
check("heading is either right (+-8 deg) or unknown -- never a wrong angle (2-4 m, 0-45 deg)",
      not wrong and total >= 30 and known >= 6, f"wrong {wrong[:4]}, known {known}/{total}")

obs = locate(render([(7, 0.15, (2.5, 0.0, 1.0), 180.0 + 82.0)]))
check("seen almost edge-on (82 deg): not located (rejected or not decoded)",
      all(o.reject is not None for o in obs), str([(o.marker_id, o.reject) for o in obs]))
obs = locate(render([(7, 0.15, (12.0, 0.0, 1.0), 180.0)]))
check("15 cm marker at 12 m (~7.6 px): not used", all(o.reject is not None for o in obs),
      str([(o.marker_id, o.reject, round(o.side_px, 1)) for o in obs]))
obs = locate(render([(7, 0.15, (3.0, 0.0, 1.0), 180.0), (3, 0.15, (4.0, -1.5, 1.2), 180.0)]))
check("two markers in one frame: both located with their own ids",
      sorted(o.marker_id for o in obs if o.reject is None) == [3, 7])

# Range source. Well resolved (>= 20 px): the marker's own range, whatever the
# LiDAR says -- it ranges what is behind or around the marker.
a = np.linspace(-0.3, 0.3, 40)
frame = render([(7, 0.175, (1.25, 0.0, 1.0), 180.0)])                      # hand-held, ~110 px
held = (1.38 + 0.2 * (1 - np.cos(a)), 0.2 * np.sin(a) * 1.5)               # the body 0.13 m behind it
o = locate(frame, size=0.175, pts=held)[0]
check("hand-held marker at 1.25 m, body 0.13 m behind on the LiDAR: the MARKER's range (live 09-30)",
      o.reject is None and o.source == "marker" and abs(o.xy[0] - 1.25) < 0.03, f"{o.source} {o.xy} {o.reject}")
frame = render([(7, 0.15, (4.0, 0.0, 1.0), 180.0)])                        # ~24 px
body = (4.02 + 0.2 * (1 - np.cos(a)), 0.2 * np.sin(a) * 1.5)
o = locate(frame, pts=body)[0]
check("marker at 4 m (24 px), LiDAR agrees: still the marker's own range",
      o.source == "marker" and abs(o.xy[0] - 4.0) < 0.15, f"{o.source} {o.xy}")
# small (< 20 px, far): the LiDAR's range, if it agrees
far = render([(7, 0.15, (6.0, 0.0, 1.0), 180.0)])                          # ~16 px
fbody = (6.02 + 0.2 * (1 - np.cos(a)), 0.2 * np.sin(a) * 1.5)
o = locate(far, pts=fbody)[0]
check("marker at 6 m (16 px), LiDAR return at its range -> range source 'lidar'",
      o.reject is None and o.source == "lidar" and abs(o.xy[0] - 6.0) < 0.1, f"{o.source} {o.xy} {o.reject}")
wall = (np.full(80, 8.5), np.linspace(-2, 2, 80))
o = locate(far, pts=wall)[0]
check("... LiDAR return 2.5 m behind it (disagrees) -> the marker's own range",
      o.source == "marker" and abs(o.xy[0] - 6.0) < 0.4, f"{o.source} {o.xy}")
# OpenCV's too-close-candidates filter lost a 6x6 marker with a one-cell margin
det6 = aruco_detector("DICT_6X6_50")
n6 = sum(any(mid == 0 for mid, _ in det6(cv2.cvtColor(
    render([(0, 0.175, (x, 0.0, 1.0), 180.0 + t)], 1, dictionary="DICT_6X6_50"), cv2.COLOR_BGR2GRAY)))
    for x in (1.2, 1.5, 2.0, 2.5, 4.0, 6.0) for t in (-16.0, 0.0, 16.0))
check("6x6 marker, 17.5 cm, one-cell white margin: read at 1.2-6 m (18 of 18)", n6 == 18, f"{n6}/18")
frame = render([(7, 0.15, (4.0, 0.0, 1.0), 180.0)])
o = locate(frame, size=0.0, pts=body)[0]
check("marker size not set: LiDAR range only", o.source == "lidar" and o.xy is not None)
o = locate(frame, size=0.0)[0]
check("marker size not set and no LiDAR: no range (shown, not selectable)", o.xy is None)
o = locate(frame, trk=MarkerTracker(allowed_ids=[1, 2]))[0]
check("id not in the allowed list: rejected", o.reject is not None and "allowed" in o.reject)


# ---------------- tracking ----------------
print("tracking:")
DT = 0.1
T0 = (0.0, 0.0, 0.0)


def ob(mid, x, y, src="marker"):
    return MarkerObs(mid, [0.0] * 8, xy=(x, y), bearing=math.atan2(y, x), heading=0.0,
                     side_px=40.0, source=src)


t = MarkerTracker()
tid = 1_000_000 + 7
t.update([ob(7, 3.0, 0.0)], 0.0, T0)
check("new marker: not selectable before 3 sightings",
      t.select(tid) == (False, "not confirmed yet (seen too few times)"))
t.update([ob(7, 3.0, 0.0)], DT, T0)
t.update([ob(7, 3.0, 0.0)], 2 * DT, T0)
check("confirmed after 3 sightings -> selectable", t.select(tid)[0] and t.state(2 * DT) == TRACKING)
_, tgt = t.update([ob(7, 3.1, 0.0)], 3 * DT, T0)
check("selected + visible -> target (base frame), source marker",
      tgt is not None and abs(tgt[0] - 3.1) < 1e-6 and tgt[2] == "marker")
_, tgt = t.update([ob(7, 3.1, 0.0, "lidar")], 4 * DT, T0)
check("... LiDAR-ranged -> source marker+lidar", tgt is not None and tgt[2] == "marker+lidar")
_, tgt = t.update([ob(3, 5.0, 1.0)], 5 * DT, T0)
check("another marker in view, ours not: no target, still TRACKING (short gap)",
      tgt is None and t.state(5 * DT) == TRACKING)
now = 5 * DT
for k in range(8):
    now += DT
    _, tgt = t.update([ob(3, 5.0, 1.0)], now, T0)
check("unseen > 0.5 s -> SEARCHING, never the other marker", t.state(now) == SEARCHING and tgt is None)
h = t.search_hint(now)
check("search hint = where it was last seen", h is not None and abs(h[0] - 3.1) < 0.3, str(h))
now += DT
_, tgt = t.update([ob(7, 3.3, 0.4)], now, T0)
check("back in view: first sighting does not resume yet (2 needed)",
      tgt is None and t.state(now) == SEARCHING)
now += DT
_, tgt = t.update([ob(7, 3.3, 0.4)], now, T0)
check("second sighting -> resumes, same id, reported as refound",
      tgt is not None and t.state(now) == TRACKING and t.refound is not None and t.refound[0] == 7)

# motion gate: a second print of the same id 4 m away
now += DT
_, tgt = t.update([ob(7, 7.5, -2.0)], now, T0)
check("same id suddenly 4 m away (another print): ignored for a frame",
      tgt is None and t.stats["gated"] >= 1)
now += DT
_, tgt = t.update([ob(7, 3.3, 0.4)], now, T0)
check("... the real one next frame is the target again", tgt is not None and abs(tgt[0] - 3.3) < 1e-6)
for k in range(3):
    now += DT
    _, tgt = t.update([ob(7, 7.5, -2.0)], now, T0)
check("... but if ONLY the far one is seen for 3 frames it is accepted (re-anchored)",
      tgt is not None and abs(tgt[0] - 7.5) < 1e-6)

# detector stall: no frames for 2 s, then the marker
now += 2.0
_, tgt = t.update([ob(7, 7.5, -2.0)], now, T0)
check("after a 2 s detector stall the first sighting does not resume by itself", tgt is None)
now += DT
_, tgt = t.update([ob(7, 7.5, -2.0)], now, T0)
check("... the second does", tgt is not None)

# window expiry
for k in range(320):
    now += DT
    t.update([], now, T0)
check("unseen > 30 s -> LOST (selection kept)", t.state(now) == LOST and t.selected_id == tid)
now += DT
t.update([ob(7, 3.0, 0.0)], now, T0)
now += DT
_, tgt = t.update([ob(7, 3.0, 0.0)], now, T0)
check("the same marker later: tracked again (ID is exact); the controller decides whether to drive",
      tgt is not None and t.state(now) == TRACKING)
t.clear()
check("clear -> NONE, no target",
      t.state(now) == NONE and t.update([ob(7, 3.0, 0.0)], now + DT, T0)[1] is None)

t = MarkerTracker()
t.update([ob(5, 3.0, 0.0)], 0.0, T0)
for k in range(1, 20):
    t.update([], k * DT, T0)
check("a one-frame decode is forgotten, never selectable",
      not t.select(1_000_000 + 5)[0] and 5 not in t.tracks)
t.update([ob(5, 3.0, 0.0), ob(5, 6.0, 0.0)], 3.0, T0)
check("the same id twice in one frame: one track, first kept",
      len(t.tracks) == 1 and abs(t.tracks[5].xy[0] - 3.0) < 1e-6 and t.stats["duplicate"] == 1)
t = MarkerTracker()
for k in range(3):
    t.update([MarkerObs(9, [0.0] * 8, xy=None, bearing=0.1, side_px=30.0)], k * DT, T0)
check("confirmed but never ranged -> cannot be selected",
      t.tracks[9].confirmed
      and t.select(1_000_000 + 9) == (False, "no range yet (marker size unset and no LiDAR return)"))
t = MarkerTracker()
t.update([MarkerObs(4, [0.0] * 8, reject="too small (8 px)")], 0.0, T0)
check("rejected sightings create no track", 4 not in t.tracks and t.stats["rejected"] == 1)

print(f"\n{sum(results)}/{len(results)} passed")
sys.exit(0 if all(results) else 1)
