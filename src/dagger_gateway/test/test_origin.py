#!/usr/bin/env python3
"""The Cube's EKF origin when the rover starts without GPS (owner, 2026-10-26). NO ROS.

    python3 src/dagger_gateway/test/test_origin.py     (or pytest)
"""
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway.origin import OriginKeeper, OriginStore  # noqa: E402

KEPT = (26.7479, 75.8553, 432.1)


def run(k, t, **kw):
    args = dict(fcu_ok=True, origin_known=False, gps_fix=False, fallback=KEPT)
    args.update(kw)
    return k.step(t, **args)


def test_booted_indoors_the_kept_origin_is_set_after_the_wait():
    k = OriginKeeper(wait_s=20.0, ask_s=5.0)
    assert run(k, 0.0) == "ask", "the Cube is asked first"
    assert run(k, 1.0) is None
    assert run(k, 5.0) == "ask"
    assert run(k, 19.9) in (None, "ask"), "GPS gets 20 s to set it itself"
    assert run(k, 20.0) == ("set", KEPT)
    assert run(k, 21.0) in (None, "ask"), "then asked until the Cube confirms"


def test_nothing_is_set_with_a_fix_with_an_origin_or_with_nothing_kept():
    k = OriginKeeper(wait_s=20.0)
    assert all(run(k, t, gps_fix=True) != ("set", KEPT) for t in range(0, 60)), "GPS sets it"
    k = OriginKeeper(wait_s=20.0)
    assert all(run(k, t, origin_known=True) is None for t in range(0, 60)), "already has one"
    k = OriginKeeper(wait_s=20.0)
    assert all(run(k, t, fallback=None) in (None, "ask") for t in range(0, 60)), "nothing to set"
    k = OriginKeeper(wait_s=20.0)
    assert all(run(k, t, fcu_ok=False) is None for t in range(0, 60)), "no Cube: nothing"


def test_set_again_if_not_confirmed_at_most_three_times():
    k = OriginKeeper(wait_s=20.0, resend_s=30.0, tries=3)
    sets = [t for t in range(0, 300) if run(k, float(t)) == ("set", KEPT)]
    assert sets == [20, 50, 80]


def test_a_cube_that_restarts_is_handled_again():
    k = OriginKeeper(wait_s=20.0)
    assert run(k, 20.0) == "ask" and run(k, 40.0) == ("set", KEPT)
    assert run(k, 41.0, fcu_ok=False) is None, "the Cube restarted: its origin is gone"
    assert run(k, 42.0) == "ask" and run(k, 62.0) == ("set", KEPT)


def test_the_origin_is_kept_across_restarts_and_a_bad_file_is_none():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "origin.json")
        assert OriginStore(path).load() is None
        OriginStore(path).save(*KEPT)
        assert OriginStore(path).load() == KEPT
        with open(path, "w") as f:
            f.write("{oops")
        assert OriginStore(path).load() is None


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok   ", name)
    print("all passed")
