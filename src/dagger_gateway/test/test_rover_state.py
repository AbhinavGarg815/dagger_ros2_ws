#!/usr/bin/env python3
"""The operating state machine, alerts and link-loss policy. NO ROS, NO SOCKETS.

    python3 src/dagger_gateway/test/test_rover_state.py     (or pytest)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway import api_pb2 as pb  # noqa: E402
from dagger_gateway.rover_state import CHECKS, RoverCore, Settings  # noqa: E402


class Actions:
    def __init__(self):
        self.calls = []

    def follow_off(self, why):
        self.calls.append(("follow_off", why))

    def pause_route(self, why):
        self.calls.append(("pause_route", why))


def make(**kw):
    events, actions = [], Actions()
    core = RoverCore(Settings(**kw), events.append, actions)
    return core, events, actions


def all_checks(status, override=None):
    c = {cid: (status, {}) for cid, _ in CHECKS}
    c.update(override or {})
    return c


def changes(events):
    return [(e.state_changed.from_state, e.state_changed.to_state, e.state_changed.reason)
            for e in events if e.WhichOneof("kind") == "state_changed"]


def active(core):
    return [a.code for a in core.alerts.sorted()]


def test_starting_then_ready_then_not_ready():
    core, ev, _ = make()
    assert core.state == pb.OPERATING_STATE_STARTING
    core.set_checks(all_checks(pb.CHECK_STATUS_PASS, {pb.CHECK_POSITION: (pb.CHECK_STATUS_PENDING, {})}))
    assert core.state == pb.OPERATING_STATE_STARTING
    core.set_checks(all_checks(pb.CHECK_STATUS_PASS))
    assert core.state == pb.OPERATING_STATE_READY
    core.set_checks(all_checks(pb.CHECK_STATUS_PASS, {pb.CHECK_CAMERA: (pb.CHECK_STATUS_FAIL, {})}))
    assert core.state == pb.OPERATING_STATE_NOT_READY, "a failing check is never READY"
    core.set_checks(all_checks(pb.CHECK_STATUS_WARN))
    assert core.state == pb.OPERATING_STATE_READY, "WARN is usable"
    assert changes(ev)[-2:] == [
        (pb.OPERATING_STATE_READY, pb.OPERATING_STATE_NOT_READY, pb.STATE_REASON_READINESS),
        (pb.OPERATING_STATE_NOT_READY, pb.OPERATING_STATE_READY, pb.STATE_REASON_READINESS)]


def test_stop_latches_and_its_alert_ends_with_it():
    core, ev, _ = make()
    core.set_checks(all_checks(pb.CHECK_STATUS_PASS))
    core.stop()
    assert core.state == pb.OPERATING_STATE_STOPPED
    assert active(core) == [pb.ALERT_STOPPED_BY_OPERATOR]
    core.set_checks(all_checks(pb.CHECK_STATUS_PASS))
    core.set_following(True)
    assert core.state == pb.OPERATING_STATE_STOPPED, "nothing overrides the latch"
    core.set_following(False)
    core.release()
    assert core.state == pb.OPERATING_STATE_READY and active(core) == []
    kinds = [e.WhichOneof("kind") for e in ev]
    assert kinds.count("alert_raised") == 1 and kinds.count("alert_cleared") == 1
    raised = next(e.alert_raised for e in ev if e.WhichOneof("kind") == "alert_raised")
    assert next(e.alert_cleared for e in ev if e.WhichOneof("kind") == "alert_cleared") == raised.id


def test_follow_off_after_silence_only_for_handshaken_apps():
    core, ev, act = make()
    core.set_checks(all_checks(pb.CHECK_STATUS_PASS))
    core.set_following(True)
    assert core.state == pb.OPERATING_STATE_FOLLOWING
    core.link_tick([], now_s=100.0)                  # no handshaken app: nothing to enforce
    core.link_tick([1499], now_s=100.0)
    assert act.calls == []
    core.link_tick([200, 1500], now_s=100.2)         # any silent app counts
    assert act.calls == [("follow_off", "app silent 1.5 s")]
    assert core.state == pb.OPERATING_STATE_READY
    assert changes(ev)[-1] == (pb.OPERATING_STATE_FOLLOWING, pb.OPERATING_STATE_READY,
                               pb.STATE_REASON_APP_SILENT)
    core.set_following(True)                         # still enabled per /follow/state (dropped msg)
    core.link_tick([1700], now_s=100.4)
    assert len(act.calls) == 1, "resent at most once a second"
    core.link_tick([2600], now_s=101.3)
    assert len(act.calls) == 2


def test_disconnect_counts_as_silent_at_once():
    core, _, act = make()
    core.set_following(True)
    core.link_tick([float("inf")], now_s=5.0)
    assert act.calls == [("follow_off", "app silent 100.0 s")]


def test_route_pauses_and_stays_paused_when_the_app_returns():
    core, ev, act = make(route_pause_ms=3000)
    core.set_checks(all_checks(pb.CHECK_STATUS_PASS))
    core.route = "running"                           # Phase 3 sets this
    core.update(pb.STATE_REASON_OPERATOR)
    core.link_tick([2999], now_s=1.0)
    assert core.state == pb.OPERATING_STATE_RUNNING_ROUTE
    core.link_tick([3000], now_s=1.2)
    assert core.state == pb.OPERATING_STATE_PAUSED and ("pause_route", "app silent") in act.calls
    assert changes(ev)[-1][2] == pb.STATE_REASON_APP_SILENT
    a = core.alerts.active[pb.ALERT_PAUSED_APP_SILENT]
    assert a.severity == pb.SEVERITY_ACTION and a.values["seconds"] == 3.0
    core.link_tick([50], now_s=2.0)                  # the app is back
    assert core.state == pb.OPERATING_STATE_PAUSED, "stays paused until the operator resumes"
    core.stop()                                      # MK32 finding: no stale 'paused' card over STOPPED
    assert core.state == pb.OPERATING_STATE_STOPPED
    assert active(core) == [pb.ALERT_STOPPED_BY_OPERATOR]


def test_route_pause_bounds_are_enforced():
    assert Settings(route_pause_ms=500).route_pause_ms == 1500
    assert Settings(route_pause_ms=60000).route_pause_ms == 10000


def test_readiness_alerts_and_order():
    core, _, _ = make()
    core.set_checks(all_checks(pb.CHECK_STATUS_PASS, {
        pb.CHECK_HEADING: (pb.CHECK_STATUS_FAIL, {"valid_percent": 42.0}),
        pb.CHECK_REMOTE_CONTROL: (pb.CHECK_STATUS_FAIL, {}),
        pb.CHECK_BATTERY: (pb.CHECK_STATUS_WARN, {"percent": 25.0})}))
    assert active(core) == [pb.ALERT_RC_LINK_LOST, pb.ALERT_HEADING_UNKNOWN, pb.ALERT_BATTERY_LOW]
    core.set_checks(all_checks(pb.CHECK_STATUS_PASS, {
        pb.CHECK_BATTERY: (pb.CHECK_STATUS_FAIL, {"percent": 9.0})}))
    assert active(core) == [pb.ALERT_BATTERY_CRITICAL]
    core.set_checks(all_checks(pb.CHECK_STATUS_PASS, {pb.CHECK_BATTERY: (pb.CHECK_STATUS_WARN, {})}))
    assert active(core) == [], "no reading is not a low battery"
    msgs = core.readiness_msgs()
    assert [m.id for m in msgs] == [cid for cid, _ in CHECKS]
    cam = msgs[-1]
    assert list(cam.blocks) == [pb.JOB_FOLLOW]


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"ok    {t.__name__}")
    print(f"\n{len(tests)} passed")
