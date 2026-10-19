#!/usr/bin/env python3
"""Step D3: when GPS counts as lost and back, RTK lost, and the autopilot disagreeing
with the receiver. NO ROS.

    python3 src/dagger_gateway/test/test_gps_watch.py     (or pytest)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway.gps_watch import DisagreeWatch, GpsWatch, RtkWatch, angle_diff_deg  # noqa: E402


def feed_for(w, t0, t1, *sample, step=0.2):
    events, t = [], t0
    while t <= t1 + 1e-9:
        ev = w.feed(t, *sample)
        if ev:
            events.append((round(t, 1), ev))
        t += step
    return events


def test_lost_after_5_s_of_no_fix_poor_accuracy_or_no_heading_and_back_after_10_s():
    for bad in ((2, 1.0, True), (3, 3.5, True), (6, 0.03, False), (None, None, False)):
        w = GpsWatch(5.0, 3.0, 10.0)
        assert feed_for(w, 0.0, 4.8, *bad) == [], f"a moment's dip is not lost: {bad}"
        assert feed_for(w, 5.0, 5.0, *bad) == [(5.0, "lost")], bad
        assert w.bad_since == 0.0
        assert feed_for(w, 6.0, 15.8, 6, 0.03, True) == [], "good, but not yet for long"
        assert feed_for(w, 16.0, 16.0, 6, 0.03, True) == [(16.0, "back")]


def test_a_dip_resets_the_clock_and_rtk_float_or_plain_3d_is_not_lost():
    w = GpsWatch(5.0, 3.0, 10.0)
    feed_for(w, 0.0, 4.0, 3, 9.0, True)
    feed_for(w, 4.2, 4.2, 6, 0.03, True)              # one good sample
    assert feed_for(w, 4.4, 9.2, 3, 9.0, True) == [], "the clock started again"
    w = GpsWatch(5.0, 3.0, 10.0)
    assert feed_for(w, 0.0, 30.0, 3, 1.5, True) == [], "plain 3-D fix at 1.5 m: not lost"


def test_rtk_lost_only_while_the_base_is_sending():
    r = RtkWatch(5.0)
    assert not any(r.feed(t, False, 3) for t in (0.0, 6.0, 12.0)), "no base: nothing to lose"
    assert not r.feed(0.0, True, 3) and not r.feed(4.0, True, 5)
    assert not r.feed(6.0, True, 4) and r.feed(11.0, True, 4)
    assert not r.feed(11.2, True, 6), "RTK fixed again"


def test_the_autopilot_disagreeing_with_gps_for_5_s():
    d = DisagreeWatch(5.0, 20.0, 3.0)
    assert angle_diff_deg(359.0, 2.0) == 3.0
    # 2026-10-19: Cube 181 deg, GNSS 52 deg, 5.7 m apart (its LiDAR source outdoors)
    assert not d.feed(0.0, True, 181.0, 52.0, 5.7) and d.feed(5.0, True, 181.0, 52.0, 5.7)
    assert round(d.deg) == 129 and d.m == 5.7
    assert not d.feed(5.2, True, 51.1, 51.8, 0.4), "agreeing again (the antenna offset)"
    assert not d.feed(20.0, False, 181.0, 52.0, 5.7) and not d.feed(30.0, False, 181.0, 52.0, 5.7), \
        "GPS lost: nothing to compare with"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
