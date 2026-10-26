#!/usr/bin/env python3
"""Live obstacles round the rover for the app (owner, 2026-10-26). NO ROS.

    python3 src/dagger_gateway/test/test_obstacles.py     (or pytest)
"""
import math
import os
import sys
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway.obstacles import as_cells, occupied_near  # noqa: E402


def grid(cells, size=400, res=0.1, origin=(-20.0, -20.0)):
    """A rolling costmap like Nav2's: size x size cells, `cells` {(col, row): value}."""
    data = [0] * (size * size)
    for (c, r), v in cells.items():
        data[r * size + c] = v
    info = NS(resolution=res, width=size, height=size, origin=NS(position=NS(x=origin[0], y=origin[1])))
    return info, data


def test_lethal_cells_near_the_rover_merged_to_20_cm():
    # a box at (5.05, 0.05) .. (5.15, 0.15) in odom: costmap cols 250-251, rows 200-201
    info, data = grid({(250, 200): 100, (251, 200): 100, (250, 201): 100, (251, 201): 100,
                       (260, 200): 99,               # inscribed margin, not an obstacle
                       (399, 399): 100})             # 28 m away
    got = occupied_near(info, data, (0.0, 0.0))
    assert len(got) == 1 and math.dist(got[0], (5.1, 0.1)) < 1e-9, got


def test_only_within_range_nearest_first_and_capped():
    cells = {(200 + k * 10, 200): 100 for k in range(0, 15)}      # every metre east, 0..14 m
    info, data = grid(cells)
    got = occupied_near(info, data, (0.0, 0.0), range_m=10.0)
    assert all(math.hypot(x, y) <= 10.0 + 0.2 for x, y in got) and len(got) == 10
    assert got[0][0] < got[-1][0], "nearest first"
    assert len(occupied_near(info, data, (0.0, 0.0), max_cells=3)) == 3


def test_points_become_whole_steps_from_the_anchor():
    assert as_cells((10.0, 20.0), [(10.0, 22.0), (9.0, 20.0), (9.02, 20.01)]) == [0, 10, -5, 0]


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok   ", name)
    print("all passed")
