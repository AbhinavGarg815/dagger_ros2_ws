#!/usr/bin/env python3
"""Live obstacles as the app gets them (owner, 2026-10-26): Gateway.tick_obstacles with stand-ins
for the ROS side. NO ROVER. Needs rclpy (the gateway module imports it): skipped where missing.

    python3 -m pytest src/dagger_gateway/test/test_obstacles_send.py
"""
import os
import sys
from types import SimpleNamespace as NS

import pytest

pytest.importorskip("rclpy")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway.gateway import Gateway  # noqa: E402


class App:
    def __init__(self):
        self.sent = []

    def send(self, **kw):
        self.sent.append(kw)


def costmap(cells, size=400, res=0.1, origin=(-20.0, -20.0)):
    data = [0] * (size * size)
    for c, r in cells:
        data[r * size + c] = 100
    info = NS(resolution=res, width=size, height=size, origin=NS(position=NS(x=origin[0], y=origin[1])))
    return NS(header=NS(frame_id="odom"), info=info, data=data)


def gateway(found=None, maps=None, pair=None, snap=None, cm=None, here=(0.0, 0.0, 0.0)):
    app = App()
    node = NS(costmap=cm if cm is not None else costmap([(250, 200), (251, 201)]), odom_now=lambda: here,
              map_pose_and_odom=lambda: pair, maps=maps, gps_snapshot=lambda: snap)
    gw = NS(apps=lambda: [app], node=node, found_pose=lambda: found, lost_snap=None, gps=NS(lost=False),
            _obstacles_ll=Gateway._obstacles_ll)
    return gw, app


def test_by_gps_cells_are_east_north_steps_round_the_rover():
    gw, app = gateway(snap=((28.5, 77.2), (0.0, 0.0), (0.0, 0.0, 0.0)))
    Gateway.tick_obstacles(gw)
    o = app.sent[0]["obstacles"]
    assert abs(o.anchor.latitude_deg - 28.5) < 1e-9 and abs(o.cell_m - 0.2) < 1e-6
    cells = list(zip(o.cells[0::2], o.cells[1::2]))
    assert len(cells) == 1 and cells[0][0] in (25, 26) and cells[0][1] in (0, 1), "the box 5.1 m east (within a cell)"


def test_on_a_map_not_placed_cells_are_picture_steps():
    m = NS(name="Workshop", placed=False, geo=[], origin_x=-5.0, origin_y=-3.0)
    gw, app = gateway(found=("Workshop", 0.0, 0.0, 0.0), maps=NS(localizing="Workshop", maps=[m]),
                      pair=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    Gateway.tick_obstacles(gw)
    o = app.sent[0]["obstacles"]
    assert (o.anchor.map_x_m, o.anchor.map_y_m) == (5.0, 3.0) and o.anchor.latitude_deg == 0.0
    cells = list(zip(o.cells[0::2], o.cells[1::2]))
    assert len(cells) == 1 and cells[0][0] in (25, 26) and cells[0][1] in (0, 1), cells


def test_nothing_without_a_place_or_a_costmap():
    for gw, app in (gateway(), gateway(snap=((28.5, 77.2), (0.0, 0.0), (0.0, 0.0, 0.0)), here=None)):
        Gateway.tick_obstacles(gw)
        assert app.sent == []
