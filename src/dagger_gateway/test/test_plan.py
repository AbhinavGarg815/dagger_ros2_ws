#!/usr/bin/env python3
"""The rover's plan as the app gets it (owner, 2026-10-23): Gateway.fill_plan with stand-ins for
the ROS side. NO ROVER. Needs rclpy (the gateway module imports it): skipped where it is missing.

    python3 -m pytest src/dagger_gateway/test/test_plan.py
"""
import math
import os
import sys
from types import SimpleNamespace as NS

import pytest

pytest.importorskip("rclpy")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway import api_pb2 as pb  # noqa: E402
from dagger_gateway.gateway import Gateway  # noqa: E402
from dagger_gateway.routes import odom_to_ll  # noqa: E402

PLAN = [(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (8.0, 2.0)]      # odom, round something in the way


def gateway(mode, plan=PLAN, snap=None, lost_snap=None, pair=None, maps=None, stone=False, batch=(1, 3),
            points=(), xy=None, odom=None):
    node = NS(fresh_plan=lambda: plan, gps_snapshot=lambda: snap, map_pose_and_odom=lambda: pair, maps=maps)
    ex = NS(mode=mode, batch=batch, stone=stone, points=list(points), xy=xy, odom=odom)
    return NS(route_exec=ex, node=node, lost_snap=lost_snap)


def test_a_gps_mission_gets_its_plan_in_lat_lon_through_the_end_of_the_drive():
    snap = ((28.5, 77.2), (10.0, 20.0), (1.0, -1.0, 0.3))
    rp = pb.RouteProgress()
    Gateway.fill_plan(gateway("gps", snap=snap), rp)
    assert rp.planned_through_index == 3
    assert len(rp.planned_path) == len(PLAN)
    for w, (x, y) in zip(rp.planned_path, PLAN):
        lat, lon = odom_to_ll(x, y, snap)
        assert abs(w.latitude_deg - lat) < 1e-12 and abs(w.longitude_deg - lon) < 1e-12


def test_gps_lost_uses_the_snapshot_from_when_it_was_lost():
    lost = ((28.5, 77.2), (0.0, 0.0), (0.0, 0.0, 0.0))
    rp = pb.RouteProgress()
    Gateway.fill_plan(gateway("odom", snap=None, lost_snap=lost), rp)
    assert len(rp.planned_path) == len(PLAN) and rp.planned_path[0].latitude_deg == 28.5


def test_a_map_not_placed_gets_its_plan_in_metres_on_the_picture():
    m = NS(name="Workshop", placed=False, geo=[], origin_x=-5.0, origin_y=-3.0)
    maps = NS(localizing="Workshop", maps=[m])
    pair = ((2.0, 1.0, 0.0), (2.0, 1.0, 0.0))                  # map and odom the same here
    rp = pb.RouteProgress()
    Gateway.fill_plan(gateway("map", pair=pair, maps=maps), rp)
    assert [(w.map_x_m, w.map_y_m) for w in rp.planned_path] == [(x + 5.0, y + 3.0) for x, y in PLAN]
    assert all(w.latitude_deg == 0.0 for w in rp.planned_path)


def test_no_fresh_plan_or_no_way_to_place_it_sends_nothing():
    for g in (gateway("gps", plan=None), gateway("gps", snap=None), gateway("map", pair=None, maps=None)):
        rp = pb.RouteProgress()
        Gateway.fill_plan(g, rp)
        assert not rp.planned_path and rp.planned_through_index == 0


def test_aiming_at_a_step_the_plan_goes_on_straight_to_the_point():
    # A long leg (stepping stones): the plan reaches the step, then straight on to point 2.
    snap = ((28.5, 77.2), (0.0, 0.0), (0.0, 0.0, 0.0))
    rp = pb.RouteProgress()
    Gateway.fill_plan(gateway("gps", snap=snap, stone=True, batch=(1, 1),
                              points=[(28.5, 77.2), (28.5003, 77.2002)]), rp)
    assert rp.planned_through_index == 2
    assert len(rp.planned_path) == len(PLAN) + 1
    assert (rp.planned_path[-1].latitude_deg, rp.planned_path[-1].longitude_deg) == (28.5003, 77.2002)
    m = NS(name="Workshop", placed=False, geo=[], origin_x=0.0, origin_y=0.0)
    rp = pb.RouteProgress()
    Gateway.fill_plan(gateway("map", pair=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), maps=NS(localizing="Workshop", maps=[m]),
                              stone=True, batch=(1, 1), xy=[(0.0, 0.0), (30.0, 4.0)]), rp)
    assert (rp.planned_path[-1].map_x_m, rp.planned_path[-1].map_y_m) == (30.0, 4.0) and rp.planned_through_index == 2


def test_the_rovers_plan_bends_where_nav2_bends():
    snap = ((0.0, 0.0), (0.0, 0.0), (0.0, 0.0, 0.0))
    rp = pb.RouteProgress()
    Gateway.fill_plan(gateway("gps", snap=snap), rp)
    north = [w.latitude_deg * 111_320.0 for w in rp.planned_path]
    assert math.isclose(max(north), 2.0, abs_tol=0.05), "the 2 m sidestep is in what the app gets"
