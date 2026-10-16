#!/usr/bin/env python3
"""Offline test of dagger_bringup.launches (the map manager's SLAM launches). NO ROS, NO ROVER.

    python3 tools/test_launches.py        (or: python3 -m pytest tools/test_launches.py)

Fake launches that take a while to stop: asking must return at once (the map
manager answers the gateway straight away -- Stop showing went unanswered on
the rover, 2026-10-16), and the worker must do the asks one at a time, in order:
localization off before mapping on, and back on the map after.
"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "dagger_bringup"))
from dagger_bringup import launches  # noqa: E402


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    error = info


class FakeProc:
    n = 0

    def __init__(self, args):
        FakeProc.n += 1
        self.pid, self.args = FakeProc.n, args


def make(stop_s=0.3):
    events, lock = [], threading.Lock()

    def launch(self, logname, args):
        p = FakeProc(args)
        with lock:
            events.append(("start", tuple(args)))
        return p

    def stop(proc):
        time.sleep(stop_s)                      # a ros2 launch takes a while to stop
        with lock:
            events.append(("stop", tuple(proc.args)))

    launches.Launches._launch = launch
    launches.stop_group = stop
    w = launches.Launches("/tmp", Log())
    return w, events


def wait_for(cond, s=5.0):
    t = time.monotonic()
    while not cond() and time.monotonic() - t < s:
        time.sleep(0.02)
    assert cond()


def test_asking_returns_at_once_and_is_done_in_order():
    w, ev = make(stop_s=0.5)
    w.localize("Yard")
    t0 = time.monotonic()
    w.localize("")                              # Stop showing
    w.map(True)                                 # New map
    w.map(False)                                # Discard / saved
    w.localize("Yard")                          # back on the shown map
    assert time.monotonic() - t0 < 0.05, "no ask waits for a launch to stop"
    wait_for(lambda: len(ev) == 5)
    loc = ("mode:=localization", "shadow:=true", "map:=" + os.path.join("/tmp", "Yard"))
    assert ev == [("start", loc), ("stop", loc), ("start", ()), ("stop", ()), ("start", loc)]


def test_the_same_map_twice_starts_one_localization():
    w, ev = make(stop_s=0.0)
    w.localize("Yard")
    w.localize("Yard")
    w.map(False)                                # not mapping: nothing to stop
    time.sleep(0.3)
    assert [e[0] for e in ev] == ["start"]


def test_switching_maps_stops_the_old_one_first():
    w, ev = make(stop_s=0.1)
    w.localize("Yard")
    w.localize("Barn")
    wait_for(lambda: len(ev) == 3)
    assert ev[1][0] == "stop" and ev[1][1][-1].endswith("Yard") and ev[2][1][-1].endswith("Barn")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok   ", name)
    print("all passed")
