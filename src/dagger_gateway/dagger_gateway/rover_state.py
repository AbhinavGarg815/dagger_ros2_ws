"""The gateway's one operating state machine, its alerts and the link-loss policy.

No ROS and no sockets here: inputs are set by ros_io (through the loop) and by the
sessions, outputs are Events handed to `emit` and actions handed to `actions`.
Everything runs on the asyncio loop thread. Tested offline by test/test_rover_state.py.

OPERATING STATE, first match wins:
    STOPPED        a STOP is latched (the app's, or /estop true from anywhere)
    PAUSED         a route is paused (by the operator, app silence, or Nav2 giving up)
    FOLLOWING      follow_controller says it is enabled
    RUNNING_ROUTE  the gateway's own route is running
    RECORDING      the operator drives, the gateway notes the route
    STARTING       a readiness check is still PENDING
    NOT_READY      a check FAILs -- never READY next to a failing check
    READY
An alert about a state ends when the state does (STOPPED_BY_OPERATOR,
PAUSED_APP_SILENT): a stale "paused" card sat over a STOPPED rover on the MK32.

LINK-LOSS (only apps that completed the handshake count; a refused or half-open
connection changes nothing): any such app silent for follow_stop_after_silence_ms
-> following off; for route_pause_after_silence_ms -> a running route pauses (reason
APP_SILENT) and stays paused when the app returns, until the operator resumes it.
An app that disconnects counts as silent at once.

JOBS (Phase 3, docs/proposals/phase3-routes.md in DaggerGCS): `may_start` says
whether a route or a recording may start and, if not, the reject reason; a new
job releases a STOP. The ROS side (route_executor, the /estop latch, the mode)
is the gateway's; here are only the rules and the state they produce.
"""
import time

from . import api_pb2 as pb

ALL_JOBS = (pb.JOB_RUN_ROUTE, pb.JOB_GO_TO_PLACE, pb.JOB_FOLLOW, pb.JOB_COME_HOME,
            pb.JOB_RECORD_ROUTE)
DRIVING_JOBS = (pb.JOB_RUN_ROUTE, pb.JOB_GO_TO_PLACE, pb.JOB_COME_HOME)

# The nine checks, in the order the app shows them, and the jobs each one blocks.
CHECKS = (
    (pb.CHECK_FLIGHT_CONTROLLER, ALL_JOBS),
    (pb.CHECK_REMOTE_CONTROL, ALL_JOBS),
    (pb.CHECK_POSITION, DRIVING_JOBS + (pb.JOB_RECORD_ROUTE,)),   # a recording is lat/lon
    (pb.CHECK_HEADING, DRIVING_JOBS),
    (pb.CHECK_OBSTACLE_SENSOR, ALL_JOBS),
    (pb.CHECK_LOCALISATION, DRIVING_JOBS),
    (pb.CHECK_NAVIGATION, DRIVING_JOBS),
    (pb.CHECK_BATTERY, ALL_JOBS),          # FAILs only below critical
    (pb.CHECK_CAMERA, (pb.JOB_FOLLOW,)),
)

# contract: highest severity first
SEVERITY_RANK = {pb.SEVERITY_PROBLEM: 0, pb.SEVERITY_ACTION: 1, pb.SEVERITY_WARNING: 2,
                 pb.SEVERITY_INFO: 3}


def wall_ms():
    return int(time.time() * 1000)


def mono_ms():
    return int(time.monotonic() * 1000)


def pending_checks():
    return {cid: (pb.CHECK_STATUS_PENDING, {}) for cid, _ in CHECKS}


class Settings:
    """RoverSettings (decided 2026-10-08). The route pause is supervisor-adjustable;
    the gateway enforces its bounds."""

    def __init__(self, heartbeat_ms=250, state_ms=200, follow_stop_ms=1500, route_pause_ms=3000,
                 route_pause_min_ms=1500, route_pause_max_ms=10000, max_speed_mps=0.6,
                 record_spacing_m=2.0):
        self.heartbeat_ms, self.state_ms = heartbeat_ms, state_ms
        self.follow_stop_ms = follow_stop_ms
        self.route_pause_min_ms, self.route_pause_max_ms = route_pause_min_ms, route_pause_max_ms
        self.route_pause_ms = self.clamp_route_pause(route_pause_ms)
        self.max_speed_mps = max_speed_mps
        self.record_spacing_m = record_spacing_m

    def clamp_route_pause(self, ms):
        return max(self.route_pause_min_ms, min(self.route_pause_max_ms, int(ms)))

    def msg(self):
        return pb.RoverSettings(
            heartbeat_period_ms=self.heartbeat_ms, state_period_ms=self.state_ms,
            follow_stop_after_silence_ms=self.follow_stop_ms,
            route_pause_after_silence_ms=self.route_pause_ms,
            route_pause_after_silence_min_ms=self.route_pause_min_ms,
            route_pause_after_silence_max_ms=self.route_pause_max_ms,
            max_speed_mps=self.max_speed_mps,
            record_point_spacing_m=self.record_spacing_m)


class Alerts:
    """Active alerts by code. Each raise and clear is also an Event."""

    def __init__(self, emit):
        self.emit = emit
        self.next_id = 1                   # unique for the life of the process
        self.active = {}                   # AlertCode -> pb.Alert

    def set(self, code, on, severity=pb.SEVERITY_WARNING, values=None):
        a = self.active.get(code)
        if on and a is None:
            a = pb.Alert(id=self.next_id, code=code, severity=severity, values=values or {},
                         raised_at_ms=wall_ms())
            self.next_id += 1
            self.active[code] = a
            self.emit(pb.Event(alert_raised=a))
        elif on:                           # still active: numbers may move, no new event
            if a.severity != severity:
                self.set(code, False)
                self.set(code, True, severity, values)
                return
            a.values.clear()
            a.values.update(values or {})
        elif a is not None:
            del self.active[code]
            self.emit(pb.Event(alert_cleared=a.id))

    def sorted(self):
        return sorted(self.active.values(), key=lambda a: (SEVERITY_RANK.get(a.severity, 9),
                                                           a.raised_at_ms, a.id))


class RoverCore:

    def __init__(self, settings, emit, actions, omit_checks=()):
        """emit(pb.Event): to every app that completed the handshake.
        actions: follow_off(why), pause_route(why) -- the ROS side.
        omit_checks: CheckIds this rover does not have (e.g. CHECK_BATTERY with no
        battery monitor): left out of readiness altogether, never a standing warning."""
        self.settings = settings
        self.emit = emit
        self.actions = actions
        self.alerts = Alerts(emit)
        self.table = tuple((cid, blocks) for cid, blocks in CHECKS if cid not in set(omit_checks))
        self.checks = {cid: (pb.CHECK_STATUS_PENDING, {}) for cid, _ in self.table}
        self.state = pb.OPERATING_STATE_STARTING
        self.stopped = False
        self.following = False
        self.route = None                  # None | "running" | "paused"
        self.pause_reason = None           # why the route is paused (a StateReason)
        self.route_id = None
        self.route_points = []
        self.target = 0                    # index of the point being driven to
        self.route_total_m = 0.0
        self.recording = None              # routes.Recorder while recording, kept after a STOP
        self.pause_silence_s = 0.0
        self.follow_off_sent = -1e9        # monotonic s: link-loss follow-off, resent at most 1/s

    # ---------------- inputs ----------------
    def set_checks(self, checks):
        """checks: CheckId -> (CheckStatus, values dict). Omitted checks are dropped."""
        ids = {cid for cid, _ in self.table}
        self.checks = {cid: v for cid, v in checks.items() if cid in ids}
        self._readiness_alerts()
        self.update(pb.STATE_REASON_READINESS)

    def set_following(self, on, reason=pb.STATE_REASON_OPERATOR):
        if on != self.following:
            self.following = on
            self.update(reason)

    def stop(self):
        """STOP: latched until released. Ends a route; a recording is kept (owner,
        2026-10-12) until the operator saves or discards it."""
        self.stopped = True
        self.route = None
        self.pause_reason = None
        self.update(pb.STATE_REASON_OPERATOR)

    def release(self):
        """/estop false (tools/estop_clear.sh): the latch is gone."""
        if self.stopped:
            self.stopped = False
            self.update(pb.STATE_REASON_OPERATOR)

    def link_tick(self, silences_ms, now_s=None):
        """silences_ms: ms since last heard, one per app that completed the handshake
        (an app that just disconnected is passed as float('inf') once)."""
        if not silences_ms:
            return
        now_s = time.monotonic() if now_s is None else now_s
        worst = max(silences_ms)
        if self.following and worst >= self.settings.follow_stop_ms \
                and now_s - self.follow_off_sent >= 1.0:
            self.follow_off_sent = now_s
            self.actions.follow_off(f"app silent {min(worst, 99999) / 1000:.1f} s")
            self.following = False         # optimistic: /follow/state confirms
            self.update(pb.STATE_REASON_APP_SILENT)
        if self.route == "running" and worst >= self.settings.route_pause_ms:
            self.actions.pause_route("app silent")
            self.route = "paused"
            self.pause_reason = pb.STATE_REASON_APP_SILENT
            self.pause_silence_s = min(worst, 99999) / 1000.0
            self.update(pb.STATE_REASON_APP_SILENT)

    # ---------------- jobs (Phase 3) ----------------
    def blocks(self, cid):
        return next((b for c, b in self.table if c == cid), ())

    def may_start(self, job):
        """None if `job` may start now, else the reject reason. A STOP does not
        prevent it: starting a job is what releases a STOP."""
        if self.recording is not None or self.route is not None or self.following:
            return pb.REJECT_REASON_NOT_IN_THIS_STATE
        for cid, (status, _) in self.checks.items():
            if status in (pb.CHECK_STATUS_FAIL, pb.CHECK_STATUS_PENDING) and job in self.blocks(cid):
                return pb.REJECT_REASON_NOT_READY
        return None

    def route_started(self, route_id, points, total_m):
        self.stopped = False
        self.route, self.pause_reason = "running", None
        self.route_id, self.route_points, self.target, self.route_total_m = route_id, list(points), 0, total_m
        self.update(pb.STATE_REASON_OPERATOR)

    def route_paused(self, reason=pb.STATE_REASON_OPERATOR):
        if self.route == "running":
            self.route, self.pause_reason = "paused", reason
            self.update(reason)

    def route_resumed(self):
        if self.route == "paused":
            self.stopped = False
            self.route, self.pause_reason = "running", None
            self.update(pb.STATE_REASON_OPERATOR)

    def route_cancelled(self):
        self.route, self.pause_reason = None, None
        self.update(pb.STATE_REASON_OPERATOR)

    def route_reached(self, index):
        """Point `index` (0-based) passed."""
        if self.route is None:
            return
        self.target = min(index + 1, max(len(self.route_points) - 1, 0))
        self.emit(pb.Event(waypoint_reached=pb.WaypointReached(route_id=self.route_id, index=index + 1)))

    def route_finished(self):
        if self.route is not None:
            self.route, self.pause_reason = None, None
            self.update(pb.STATE_REASON_ROUTE_FINISHED)

    def route_failed(self):
        """Nav2 refused or gave up: paused where it is, with "can't find a way"."""
        if self.route == "running":
            self.route, self.pause_reason = "paused", pb.STATE_REASON_PATH_BLOCKED
            self.update(pb.STATE_REASON_PATH_BLOCKED)

    def recording_started(self, recorder):
        self.stopped = False
        self.recording = recorder
        self.update(pb.STATE_REASON_OPERATOR)

    def recording_ended(self):
        """Saved or discarded."""
        self.recording = None
        self.update(pb.STATE_REASON_OPERATOR)

    # ---------------- state ----------------
    def derive(self):
        if self.stopped:
            return pb.OPERATING_STATE_STOPPED
        if self.route == "paused":
            return pb.OPERATING_STATE_PAUSED
        if self.following:
            return pb.OPERATING_STATE_FOLLOWING
        if self.route == "running":
            return pb.OPERATING_STATE_RUNNING_ROUTE
        if self.recording is not None:
            return pb.OPERATING_STATE_RECORDING
        statuses = [s for s, _ in self.checks.values()]
        if pb.CHECK_STATUS_PENDING in statuses:
            return pb.OPERATING_STATE_STARTING
        if pb.CHECK_STATUS_FAIL in statuses:
            return pb.OPERATING_STATE_NOT_READY
        return pb.OPERATING_STATE_READY

    def update(self, reason):
        new = self.derive()
        if new != self.state:
            old, self.state = self.state, new
            self.emit(pb.Event(state_changed=pb.StateChange(from_state=old, to_state=new,
                                                            reason=reason)))
        self.alerts.set(pb.ALERT_STOPPED_BY_OPERATOR, self.state == pb.OPERATING_STATE_STOPPED,
                        pb.SEVERITY_INFO)
        paused = self.state == pb.OPERATING_STATE_PAUSED
        self.alerts.set(pb.ALERT_PAUSED_APP_SILENT, paused and self.pause_reason == pb.STATE_REASON_APP_SILENT,
                        pb.SEVERITY_ACTION, {"seconds": round(self.pause_silence_s, 1)})
        self.alerts.set(pb.ALERT_NO_PATH, paused and self.pause_reason == pb.STATE_REASON_PATH_BLOCKED,
                        pb.SEVERITY_PROBLEM)

    def _readiness_alerts(self):
        st = {cid: s for cid, (s, _) in self.checks.items()}
        fail = lambda cid: st.get(cid) == pb.CHECK_STATUS_FAIL  # noqa: E731
        self.alerts.set(pb.ALERT_POSITION_NOT_ACCURATE, fail(pb.CHECK_POSITION))
        self.alerts.set(pb.ALERT_HEADING_UNKNOWN, fail(pb.CHECK_HEADING))
        self.alerts.set(pb.ALERT_RC_LINK_LOST, fail(pb.CHECK_REMOTE_CONTROL), pb.SEVERITY_PROBLEM)
        bstat, bval = self.checks.get(pb.CHECK_BATTERY, (pb.CHECK_STATUS_PENDING, {}))
        pct = {"percent": bval["percent"]} if "percent" in bval else {}
        self.alerts.set(pb.ALERT_BATTERY_CRITICAL, bstat == pb.CHECK_STATUS_FAIL and bool(pct),
                        pb.SEVERITY_PROBLEM, pct)
        self.alerts.set(pb.ALERT_BATTERY_LOW, bstat == pb.CHECK_STATUS_WARN and bool(pct),
                        pb.SEVERITY_WARNING, pct)

    def readiness_msgs(self):
        out = []
        for cid, blocks in self.table:
            status, values = self.checks.get(cid, (pb.CHECK_STATUS_PENDING, {}))
            out.append(pb.ReadinessCheck(id=cid, status=status, blocks=blocks, values=values))
        return out
