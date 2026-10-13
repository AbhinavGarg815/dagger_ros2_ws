#!/usr/bin/env python3
"""Saved routes, the recorder and the route arithmetic. NO ROS, NO SOCKETS.

    python3 src/dagger_gateway/test/test_routes.py     (or pytest)
"""
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway.routes import (BLOCKED_COST, Recorder, RouteStore, Trail, clear_spot,  # noqa: E402
                                   distance_m, home_path, ll_to_en, mission_waypoints,
                                   path_length, progress, window)

LAT0, LON0 = 26.7480, 75.8553   # near the yard (Jaipur)


def north(m, east=0.0):
    """A point `m` metres north (and `east` metres east) of LAT0, LON0."""
    return (LAT0 + m / 110_800.0, LON0 + east / (111_320.0 * 0.8930))


def test_ll_to_en_is_metres():
    e, n = ll_to_en(*north(10.0), LAT0, LON0)
    assert abs(n - 10.0) < 0.05 and abs(e) < 0.01
    assert abs(distance_m(north(0), north(0, east=5.0)) - 5.0) < 0.05


def test_store_saves_reloads_and_names():
    with tempfile.TemporaryDirectory() as d:
        store = RouteStore(d)
        assert store.all() == [] and store.suggested_name() == "Route 1"
        r = store.save("Yard loop", [north(0), north(5), north(10)])
        assert store.name_taken("Yard loop") and not store.name_taken("Gate")
        assert store.suggested_name() == "Route 2"
        again = RouteStore(d)                       # a gateway restart
        assert [x.name for x in again.all()] == ["Yard loop"]
        s = again.get(r.id).summary()
        assert len(s.waypoints) == 3 and abs(s.length_m - 10.0) < 0.1
        assert not [f for f in os.listdir(d) if f.endswith(".tmp")], "written atomically"


def test_mission_from_the_autopilot_keeps_only_waypoints():
    # As QGC leaves it on ArduPilot: home (item 0), a speed, 3 waypoints (one a
    # spline), a loiter, a waypoint in a local frame, return home.
    home, a, b, c = north(0), north(5), north(10), north(10, east=5.0)
    items = [(0, 16, *home), (3, 178, 0.0, 0.0), (3, 16, *a), (6, 82, *b), (3, 17, *b),
             (1, 16, 3.0, 4.0), (3, 16, *c), (3, 20, 0.0, 0.0)]
    assert mission_waypoints(items) == [a, b, c]
    assert mission_waypoints([(0, 16, *home)]) == [], "home alone is no mission"
    assert mission_waypoints([]) == []


def test_trail_keeps_a_point_every_metre():
    t = Trail(north(0))
    for m in (0.3, 0.6, 1.1, 1.5, 2.2):
        t.add(*north(m))
    assert len(t.points) == 3          # home, 1.1, 2.2
    t.reset()
    assert t.points == [north(0)]


def test_the_way_home_goes_back_along_the_trail_skipping_loops():
    # Out 20 m north, a loop out east and back, then 10 m more north.
    trail = [north(m) for m in range(0, 21)]
    trail += [north(20, east=e) for e in range(1, 6)] + [north(20, east=e) for e in range(4, -1, -1)]
    trail += [north(m) for m in range(21, 31)]
    path = home_path(trail, north(30.5))
    assert path[-1] == north(0), "ends exactly at home"
    assert all(abs(distance_m(p, north(0)) - 0) < 31 for p in path)
    assert all(ll_to_en(p[0], p[1], LAT0, LON0)[0] < 1.0 for p in path), "the loop east is skipped"
    gaps = [distance_m(a, b) for a, b in zip(path, path[1:])]
    assert max(gaps) < 4.5 and min(gaps[:-1]) >= 2.9, gaps
    assert home_path([north(0)], north(0.2)) == [north(0)], "already home: just home"


def test_a_waypoint_on_an_obstacle_moves_to_clear_ground_or_is_blocked():
    # a 21 x 21 grid, 0.1 m cells: lethal wall in columns 8..12, free elsewhere
    def cost_at(i, j):
        if not (0 <= i < 21 and 0 <= j < 21):
            return -1
        return 100 if 8 <= i <= 12 else 0
    assert clear_spot(cost_at, 3, 10, 10) == (3, 10), "clear: stays"
    assert clear_spot(cost_at, 8, 10, 10) == (7, 10), "on the edge: nearest clear cell"
    assert clear_spot(lambda i, j: 100, 5, 5, 10) is None, "nothing clear within 1 m: blocked"
    assert clear_spot(lambda i, j: -1, 5, 5, 10) == (5, 5), "unknown: not checked"
    assert BLOCKED_COST == 99


def test_store_skips_a_damaged_file():
    with tempfile.TemporaryDirectory() as d:
        open(os.path.join(d, "bad.json"), "w").write("{not json")
        assert RouteStore(d).all() == []


def test_recorder_keeps_a_point_every_spacing_and_at_turns():
    rec = Recorder(2.0)
    for m in [0, 0.5, 1.0, 1.9, 2.1, 3.0, 4.2]:
        rec.add(*north(m))
    assert len(rec.points) == 3, rec.points            # 0, 2.1, 4.2
    rec.add(*north(4.2, east=0.8))                      # a sharp turn, 0.8 m on
    assert len(rec.points) == 4
    assert abs(rec.length_m() - (4.2 + 0.8)) < 0.1


def test_progress_and_window():
    pts = [north(m) for m in (0, 10, 20, 30, 40)]
    total = path_length(pts)
    frac, left = progress(pts, 2, north(15), total)
    assert abs(left - 25.0) < 0.2 and abs(frac - 0.375) < 0.01
    assert window(pts, 0, north(0), 15.0) == (0, 2)      # 0 and 10 m: within 15 m
    assert window(pts, 2, north(15), 15.0) == (2, 4)     # 20 and 30 m from the rover at 15 m
    assert window(pts, 3, north(0), 15.0) == (3, 4), "a far point is still tried, alone"


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"ok    {t.__name__}")
    print(f"\n{len(tests)} passed")
