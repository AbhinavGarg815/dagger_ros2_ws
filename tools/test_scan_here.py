#!/usr/bin/env python3
""""I am here" (Step D2): MapMatcher.search_here on made-up rooms. NO ROS.

    python3 tools/test_scan_here.py          (or pytest)

A room is drawn into a .pgm/.yaml as map_saver writes them; the rover's scan is
ray-cast from where it really stands. The operator's tap is a little off and gives
no direction: the search must find the rover and its heading in a room that only
fits one way, and say "turned" (ask for the front) in a square room that fits four.
"""
import math
import os
import sys
import tempfile
import time

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "dagger_bringup"))
from dagger_bringup.scan_map_match import FOUND, NEED_FRONT, NOT_FOUND, MapMatcher, here_verdict  # noqa: E402

RES = 0.05
ORIGIN = (-2.0, -3.0)          # the picture's bottom-left on the map, as a .yaml says


def room(folder, w_m, h_m, walls):
    """A map w_m x h_m: free inside, an outer wall, and `walls` (x1, y1, x2, y2) in metres."""
    w, h = int(w_m / RES), int(h_m / RES)
    img = Image.new("L", (w, h), 254)
    d = ImageDraw.Draw(img)
    px = lambda x, y: ((x - ORIGIN[0]) / RES, h - 1 - (y - ORIGIN[1]) / RES)  # noqa: E731
    x0, y0, x1, y1 = ORIGIN[0] + 0.5, ORIGIN[1] + 0.5, ORIGIN[0] + w_m - 0.5, ORIGIN[1] + h_m - 0.5
    for a, b in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
        d.line([px(*a), px(*b)], fill=0, width=3)
    for x1_, y1_, x2_, y2_ in walls:
        d.line([px(x1_, y1_), px(x2_, y2_)], fill=0, width=3)
    img.save(os.path.join(folder, "m.pgm"))
    with open(os.path.join(folder, "m.yaml"), "w") as f:
        f.write(f"image: m.pgm\nresolution: {RES}\norigin: [{ORIGIN[0]}, {ORIGIN[1]}, 0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    # forward slashes: MapMatcher finds the .pgm beside the .yaml by the last "/"
    return MapMatcher(os.path.join(folder, "m.yaml").replace(os.sep, "/")), np.asarray(img) < 100


def scan(occ, x, y, yaw, n=360, max_r=20.0):
    """Ray-cast from (x, y, yaw): points in the rover's frame, as scan_points gives them."""
    h = occ.shape[0]
    pts = []
    for k in range(n):
        a = 2 * math.pi * k / n
        c, s = math.cos(yaw + a), math.sin(yaw + a)
        r = 0.1
        while r < max_r:
            u = int((x + c * r - ORIGIN[0]) / RES)
            v = h - 1 - int((y + s * r - ORIGIN[1]) / RES)
            if not (0 <= v < h and 0 <= u < occ.shape[1]):
                break
            if occ[v, u]:
                pts.append((r * math.cos(a), r * math.sin(a)))
                break
            r += RES / 2
    noise = np.random.default_rng(7).normal(0.0, 0.02, (len(pts), 2))   # ~2 cm, as a real LiDAR
    return (np.asarray(pts) + noise)[::max(1, len(pts) // 300)].astype(np.float32)


def apart_deg(a, b):
    return abs(math.degrees((a - b + math.pi) % (2 * math.pi) - math.pi))


def test_a_room_that_fits_one_way_finds_the_rover_and_its_heading():
    with tempfile.TemporaryDirectory() as d:
        # 12 x 8 m with a partition and a bench: no two ways round look alike
        m, occ = room(d, 12.0, 8.0, [(3.0, -2.5, 3.0, 1.5), (5.0, 2.0, 7.5, 2.0)])
        truth = (0.5, 0.0, math.radians(37))
        pts = scan(occ, *truth)
        t0 = time.monotonic()
        score, x, y, yaw, second, turned = m.search_here(pts, truth[0] + 0.6, truth[1] - 0.4, 2.5)
        took = time.monotonic() - t0
        assert math.hypot(x - truth[0], y - truth[1]) < 0.15, (x, y)
        assert apart_deg(yaw, truth[2]) < 3.0, math.degrees(yaw)
        assert here_verdict(score, second, turned, False) == FOUND, (score, second, turned)
        print(f"  found at ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg), score {score:.3f}, "
              f"elsewhere {second:.3f}, turned {turned:.3f} [{took:.1f} s on this PC]")


def test_a_square_room_asks_for_the_front_and_then_finds_it():
    with tempfile.TemporaryDirectory() as d:
        m, occ = room(d, 9.0, 9.0, [])                # a square: four ways round fit
        cx, cy = ORIGIN[0] + 4.5, ORIGIN[1] + 4.5
        truth = (cx, cy, math.radians(20))
        pts = scan(occ, *truth)
        score, x, y, yaw, second, turned = m.search_here(pts, cx + 0.3, cy, 2.5)
        assert here_verdict(score, second, turned, False) == NEED_FRONT, ("must ask", score, second, turned)
        # the operator's second tap, in front: only +-30 deg about it
        score, x, y, yaw, second, turned = m.search_here(pts, cx + 0.3, cy, 2.5, truth[2] + math.radians(10),
                                                         math.radians(30))
        assert apart_deg(yaw, truth[2]) < 3.0 and math.hypot(x - cx, y - cy) < 0.15
        assert here_verdict(score, second, turned, True) == FOUND


def test_a_tap_far_from_the_rover_is_not_found():
    with tempfile.TemporaryDirectory() as d:
        m, occ = room(d, 12.0, 8.0, [(3.0, -2.5, 3.0, 1.5), (5.0, 2.0, 7.5, 2.0)])
        pts = scan(occ, 0.5, 0.0, math.radians(37))
        score, x, y, yaw, second, turned = m.search_here(pts, 6.0, -1.5, 2.5)   # the other room
        assert here_verdict(score, second, turned, False) == NOT_FOUND, (score, second)


def test_the_verdict_on_its_own():
    assert here_verdict(0.06, 0.40, 0.50, False) == FOUND
    assert here_verdict(0.30, 0.90, 0.90, False) == NOT_FOUND, "poor fit"
    assert here_verdict(0.06, 0.08, 0.50, False) == NOT_FOUND, "another place fits as well"
    assert here_verdict(0.06, 0.40, 0.07, False) == NEED_FRONT
    assert here_verdict(0.06, 0.40, 0.07, True) == FOUND, "the operator said which way"
    assert here_verdict(0.00, 0.40, 0.03, False) == NEED_FRONT, "a perfect fit still asks"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"ok  {name}")
