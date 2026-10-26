#!/usr/bin/env python3
"""Told to drive, not moving: pause and say why (owner, 2026-10-26). NO ROS.

    python3 src/dagger_gateway/test/test_motion_watch.py     (or pytest)
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway import api_pb2 as pb  # noqa: E402
from dagger_gateway.motion_watch import (CAUSE_HELD_BACK, CAUSE_HOLD, CAUSE_NOT_ARMED, CAUSE_UNKNOWN,  # noqa: E402
                                         MotionWatch, commanding, not_moving_cause)
from dagger_gateway.rover_state import CHECKS, RoverCore, Settings  # noqa: E402

STILL = (1.0, 2.0, 0.5)


def test_asked_to_drive_without_moving_for_5_s():
    w = MotionWatch(wait_s=5.0)
    assert not w.step(0.0, True, STILL)
    assert not w.step(4.9, True, STILL)
    assert w.step(5.0, True, STILL), "5 s asked, not moved"


def test_moving_or_turning_starts_the_wait_again():
    w = MotionWatch(wait_s=5.0)
    w.step(0.0, True, STILL)
    assert not w.step(4.0, True, (1.0, 2.15, 0.5)), "15 cm on"
    assert not w.step(8.5, True, (1.0, 2.15, 0.5)), "4.5 s since it last moved"
    assert not w.step(8.9, True, (1.0, 2.15, 0.5 + math.radians(6))), "a pivot turn is moving"
    assert w.step(13.9, True, (1.0, 2.15, 0.5 + math.radians(6)))


def test_not_asked_or_no_pose_is_never_not_moving():
    w = MotionWatch(wait_s=5.0)
    assert not any(w.step(float(t), False, STILL) for t in range(0, 20)), "an obstacle stop: no command"
    assert not any(w.step(float(t), True, None) for t in range(0, 20))
    w.step(0.0, True, STILL)
    w.step(4.0, False, STILL)
    assert not w.step(6.0, True, STILL), "a pause in the asking starts the wait again"


def test_commanding_and_the_cause_in_order():
    assert commanding(0.4, 0.0) and commanding(0.0, 0.44) and not commanding(0.01, 0.05)
    assert not_moving_cause(False, "HOLD", False) == CAUSE_NOT_ARMED
    assert not_moving_cause(True, "HOLD", True) == CAUSE_HOLD
    assert not_moving_cause(True, "GUIDED", False) == CAUSE_HELD_BACK, "11:27: /cmd_vel_out silent"
    assert not_moving_cause(True, "GUIDED", True) == CAUSE_UNKNOWN, "11:09: no EKF origin"


def core_running():
    core = RoverCore(Settings(), lambda e: None, None)
    core.set_checks({cid: (pb.CHECK_STATUS_PASS, {}) for cid, _ in CHECKS})
    core.route = "running"
    core.update(pb.STATE_REASON_OPERATOR)
    return core


def test_the_mission_pauses_and_the_alert_says_why():
    core = core_running()
    core.route_not_moving(CAUSE_HOLD)
    assert core.state == pb.OPERATING_STATE_PAUSED and core.pause_reason == pb.STATE_REASON_NOT_MOVING
    a = core.alerts.active[pb.ALERT_NOT_MOVING]
    assert a.severity == pb.SEVERITY_ACTION and a.values["cause"] == CAUSE_HOLD
    assert pb.ALERT_ARM_WITH_SWITCH not in core.alerts.active
    core.route_resumed()
    assert pb.ALERT_NOT_MOVING not in core.alerts.active, "carry on clears it"


def test_not_armed_asks_for_the_arm_switch_instead():
    core = core_running()
    core.route_not_moving(CAUSE_NOT_ARMED)
    assert pb.ALERT_ARM_WITH_SWITCH in core.alerts.active and pb.ALERT_NOT_MOVING not in core.alerts.active
    core.stop()
    assert pb.ALERT_ARM_WITH_SWITCH not in core.alerts.active, "STOP ends it"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok   ", name)
    print("all passed")
