#!/usr/bin/env python3
"""Offline test of making a map (Step C2): the live Earth tie (map_tie) and the
map files written from slam_toolbox's grid (map_store). NO ROS, NO ROVER.

    python3 tools/test_map_tie.py        (or: python3 -m pytest tools/test_map_tie.py)

A pretend drive: the SLAM frame turned 30 degrees and shifted from ENU; the fit
must find the turn from the heading and the shift from the fixes, and the map
saved from a grid must come back from MapStore placed where the fit says.
"""
import math
import os
import random
import sys
import tempfile
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "dagger_bringup"))
from dagger_bringup.geo_map import GeoMap, enu_scale  # noqa: E402
from dagger_bringup.map_store import MapStore, grid_to_pgm, save_grid  # noqa: E402
from dagger_bringup.map_tie import MIN_PAIRS, Tie, geo_yaml, to_latlon  # noqa: E402

LAT0, LON0 = 26.7479, 75.8553
ROT, TX, TY = math.radians(30.0), 4.0, -2.0        # map -> ENU, the truth


def drive(tie, n=60, sigma=1.0, heading=True, noise=0.5):
    random.seed(1)
    kx, ky = enu_scale(LAT0)
    c, s = math.cos(ROT), math.sin(ROT)
    for i in range(n):
        t = i * 1.0
        x, y, yaw = 0.5 * i, 3.0 * math.sin(i / 8.0), 0.2 * math.sin(i / 5.0)   # in the map frame
        e, nn = c * x - s * y + TX, s * x + c * y + TY
        tie.add_fix(t + 0.05, LAT0 + (nn + random.gauss(0, noise)) / ky, LON0 + (e + random.gauss(0, noise)) / kx, sigma)
        if heading:
            tie.add_heading(t + 0.03, yaw + ROT)
        tie.add_pose(t, x, y, yaw)


def test_the_fit_finds_the_turn_and_the_shift():
    tie = Tie()
    drive(tie)
    fit = tie.fit()
    assert fit is not None
    assert abs(math.degrees(fit["rot"]) - 30.0) < 0.01, "the turn comes from the heading"
    # The datum is the fixes' mean, so check where a map point lands instead of t itself.
    kx, ky = enu_scale(LAT0)
    lat, lon = to_latlon(fit, 10.0, 0.0)
    e, n = (lon - LON0) * kx, (lat - LAT0) * ky
    assert math.hypot(e - (math.cos(ROT) * 10 + TX), n - (math.sin(ROT) * 10 + TY)) < 0.3
    assert fit["rms"] < 1.5


def test_an_outdoor_then_indoor_drive_is_placed_by_its_good_outdoor_part():
    # Map 4, 2026-10-23: a minute in the open with RTK and a good heading, then through a door:
    # the SLAM heading drifts up to 15 deg indoors while GPS still gives fixes a few metres off
    # (sigma 3-6 m) and the odd heading. Only the good outdoor part may place the map.
    random.seed(2)
    tie = Tie()
    kx, ky = enu_scale(LAT0)
    c, s = math.cos(ROT), math.sin(ROT)
    for i in range(160):
        t = i * 1.0
        x, y = 0.4 * i, 2.0 * math.sin(i / 9.0)
        indoors = i >= 60
        drift = math.radians(15.0) * min(1.0, (i - 60) / 60.0) if indoors else 0.0
        true_yaw = 0.2 * math.sin(i / 5.0)
        e, n = c * x - s * y + TX, s * x + c * y + TY
        if indoors:
            sig = random.uniform(3.0, 6.0)
            tie.add_fix(t + 0.05, LAT0 + (n + 8.0 + random.gauss(0, 2)) / ky, LON0 + (e - 6.0) / kx, sig)
            if i % 3 == 0:
                tie.add_heading(t + 0.03, true_yaw + ROT + math.radians(random.uniform(-20, 20)))
        else:
            tie.add_fix(t + 0.05, LAT0 + (n + random.gauss(0, 0.02)) / ky, LON0 + (e + random.gauss(0, 0.02)) / kx, 0.03)
            tie.add_heading(t + 0.03, true_yaw + ROT)
        tie.add_pose(t, x, y, true_yaw + drift)          # SLAM's own heading drifts indoors
    fit = tie.fit()
    assert fit is not None
    assert abs(math.degrees(fit["rot"]) - 30.0) < 0.5, math.degrees(fit["rot"])
    lat, lon = to_latlon(fit, 20.0, 1.0)
    e, n = (lon - LON0) * kx, (lat - LAT0) * ky
    assert math.hypot(e - (c * 20 - s * 1 + TX), n - (s * 20 + c * 1 + TY)) < 0.3
    assert fit["rms"] < 0.3, "rms over the good pairs, not inflated by indoor fixes"


def test_not_placed_without_heading_or_with_too_few_fixes():
    t1 = Tie()
    drive(t1, heading=False)
    assert t1.fit() is None, "no heading (indoors, or one antenna): not placed"
    t2 = Tie()
    drive(t2, sigma=12.0)
    assert t2.fit() is None, "fixes worse than 8 m are left out"
    t3 = Tie()
    drive(t3, n=MIN_PAIRS - 1)
    assert t3.fit() is None


def test_the_geo_file_reads_back_the_same():
    tie = Tie()
    drive(tie)
    fit = tie.fit()
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "x_geo.yaml")
        open(p, "w").write(geo_yaml(fit, "test"))
        g = GeoMap(p)
        for x, y in ((0, 0), (12.5, -7.0), (-3.0, 20.0)):
            a, b = g.to_latlon(x, y), to_latlon(fit, x, y)
            assert abs(a[0] - b[0]) < 1e-8 and abs(a[1] - b[1]) < 1e-8


def grid(w=40, h=20, res=0.05, ox=-1.0, oy=-0.5):
    data = np.full((h, w), -1, np.int8)            # row 0 = the bottom, as nav_msgs
    data[2:, :] = 0                                # seen and free, except the two bottom rows
    data[h - 1, :] = 100                           # a wall along the top
    return NS(info=NS(width=w, height=h, resolution=res, origin=NS(position=NS(x=ox, y=oy))),
              data=data.flatten().tolist())


def test_a_grid_becomes_the_files_map_saver_writes():
    v = grid_to_pgm(grid())
    assert v.shape == (20, 40)
    assert (v[0] == 0).all(), "the top row is the wall (the grid's last row)"
    assert (v[-1] == 205).all() and (v[5] == 254).all()
    tie = Tie()
    drive(tie)
    fit = tie.fit()
    with tempfile.TemporaryDirectory() as d:
        save_grid(d, "Barn", grid())
        open(os.path.join(d, "Barn_geo.yaml"), "w").write(geo_yaml(fit, "test"))
        store = MapStore(d)
        store.scan()
        m = store.maps["Barn"]
        assert (m.width_px, m.height_px) == (40, 20) and abs(m.width_m - 2.0) < 1e-9
        bl = to_latlon(fit, -1.0, -0.5)
        assert abs(m.corners[3][0] - bl[0]) < 1e-9 and abs(m.corners[3][1] - bl[1]) < 1e-9


def test_names_are_safe_and_suggested():
    with tempfile.TemporaryDirectory() as d:
        store = MapStore(d)
        store.scan()
        assert store.safe_name("  Barn/../x  ") == "Barn----x"
        assert store.suggested_name() == "Map 1"
        save_grid(d, "Map 1", grid())
        store.scan()
        assert store.name_taken("Map 1") and store.suggested_name() == "Map 2"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok   ", name)
    print("all passed")
