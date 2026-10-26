#!/usr/bin/env python3
"""Which route tree a Nav2 goal asks for (owner, 2026-10-26: indoors with no GPS since boot, every
map mission "can't find a way" -- the default tree looks the rover up in the GPS map frame).
NO ROVER; needs ROS (rclpy, nav2_msgs, the installed dagger_bringup): skipped where missing.

    python3 -m pytest src/dagger_gateway/test/test_route_tree.py
"""
import os
import sys
from types import SimpleNamespace as NS

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway.route_executor import RouteExecutor, odom_tree  # noqa: E402


def test_the_odom_tree_is_installed_and_looks_the_rover_up_in_odom():
    p = odom_tree()
    assert p is not None and os.path.exists(p)
    xml = open(p, encoding="utf-8").read()
    assert 'global_frame="odom"' in xml and 'robot_base_frame="base_footprint"' in xml


def test_goals_in_odom_ask_for_the_odom_tree_and_gps_goals_for_the_default():
    ex = NS(odom_bt="/x/dagger_navigate_through_poses_odom.xml")
    assert RouteExecutor._goal(ex, [], "odom").behavior_tree == ex.odom_bt
    assert RouteExecutor._goal(ex, [], "map").behavior_tree == ""
    assert RouteExecutor._goal(NS(odom_bt=None), [], "odom").behavior_tree == "", "not installed: the default"
