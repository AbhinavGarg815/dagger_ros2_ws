#!/usr/bin/env python3
"""Following: tracker messages -> the contract's Targets and FollowStatus. NO ROS.

    python3 src/dagger_gateway/test/test_follow.py     (or pytest)
"""
import math
import os
import sys
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway import api_pb2 as pb  # noqa: E402
from dagger_gateway.follow import follow_enabled, follow_status, is_selectable, targets_msg  # noqa: E402


def header(t):
    return NS(stamp=NS(sec=int(t), nanosec=int((t % 1) * 1e9)))


def person(i, ranged=True, rng=4.2, visible=True):
    return NS(id=i, x1=100.0, y1=50.0, x2=200.0, y2=400.0, ranged=ranged,
              range_m=rng if ranged else math.nan, visible=visible)


def marker(i, confirmed=True):
    return NS(id=i, corners=[1.0, 2, 3, 4, 5, 6, 7, 8], confirmed=confirmed, range_m=6.1, visible=True)


def tracks(sel=-1, state="NONE", people=None):
    return NS(header=header(1000.25), image_width=640, image_height=480, image_rotation_deg=180,
              selected_id=sel, target_state=state, tracks=people if people is not None else [person(7), person(8, ranged=False)])


def markers(sel=-1, state="NONE"):
    return NS(header=header(1000.25), image_width=640, image_height=480, image_rotation_deg=180,
              selected_id=sel, target_state=state, tracks=[marker(1000), marker(1001, confirmed=False)])


def test_targets_carry_people_and_markers_with_what_can_be_picked():
    t = targets_msg(tracks(), markers())
    assert (t.width, t.height, t.rotation_deg) == (640, 480, 180)
    assert abs(t.stamp_s - 1000.25) < 1e-6 and t.selected_id == -1
    by = {x.id: x for x in t.targets}
    assert by[7].kind == pb.TARGET_KIND_PERSON and list(by[7].outline) == [100, 50, 200, 400]
    assert by[7].selectable and abs(by[7].range_m - 4.2) < 1e-6
    assert not by[8].selectable and by[8].range_m == 0, "not ranged yet: cannot be picked, range unknown"
    assert by[1000].kind == pb.TARGET_KIND_MARKER and len(by[1000].outline) == 8 and by[1000].selectable
    assert not by[1001].selectable, "a marker seen too rarely cannot be picked"


def test_either_tracker_alone_is_enough():
    assert [x.id for x in targets_msg(None, markers()).targets] == [1000, 1001]
    assert [x.id for x in targets_msg(tracks(), None).targets] == [7, 8]
    empty = targets_msg(None, None)
    assert len(empty.targets) == 0 and empty.width == 640


def test_follow_status_says_who_is_picked_and_how_it_is_seen():
    assert follow_status(tracks(), markers()) is None
    s = follow_status(tracks(sel=7, state="HOLD_LIDAR"), markers())
    assert s.target_id == 7 and s.state == pb.TARGET_STATE_HOLD and abs(s.range_m - 4.2) < 1e-6
    s = follow_status(tracks(), markers(sel=1000, state="TRACKING"))
    assert s.target_id == 1000 and s.state == pb.TARGET_STATE_TRACKING
    s = follow_status(tracks(sel=9, state="LOST", people=[]), None)
    assert s.target_id == 9 and s.state == pb.TARGET_STATE_LOST and s.range_m == 0


def test_only_selectable_targets_may_be_picked():
    assert is_selectable(tracks(), markers(), 7)
    assert not is_selectable(tracks(), markers(), 8)
    assert is_selectable(tracks(), markers(), 1000)
    assert not is_selectable(tracks(), markers(), 1001)
    assert not is_selectable(tracks(), markers(), 42)


def test_a_stop_does_not_look_like_following():
    # Owner, 2026-10-26: after STOP, /follow/state said "estop" and every new job was refused.
    assert follow_enabled("following", None) and follow_enabled("searching", None)
    assert follow_enabled("no target", None)
    assert not follow_enabled("disabled: app off", None)
    assert not follow_enabled("estop", None), "nothing before the estop: not following"
    assert not follow_enabled("estop", "disabled: STOP (gateway)"), "STOP turned it off"
    assert not follow_enabled("estop", "disabled: never enabled")
    assert follow_enabled("estop", "following"), "a STOP from elsewhere while following: still on"


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"ok    {t.__name__}")
    print(f"\n{len(tests)} passed")
