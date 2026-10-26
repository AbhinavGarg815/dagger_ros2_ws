"""Told to drive, not moving (owner, 2026-10-26): pause the mission and say why.

The handset said *Driving* while the rover sat still -- the autopilot on hold, or without an
EKF origin, or its drive commands held back by a STOP lock -- until Nav2's own progress check
gave up 25 s later with a misleading "can't find a way". Now, while a mission runs and Nav2's
command (after the collision monitor: /cmd_vel_nav, so an obstacle stop is not counted) asks for
motion for `wait_s` without the rover moving, the gateway pauses it and names the cause.
"""
import math

NOT_MOVING_S = 5.0          # owner's choice, 2026-10-26 (rover setting not_moving_s)
CMD_V_MPS = 0.05            # a command asks for motion above these
CMD_W_RADS = 0.10
MOVED_M = 0.10              # the rover has moved once it went this far or turned this much
TURNED_DEG = 5.0

# Why it did not move (ALERT_NOT_MOVING "cause"; not armed is ALERT_ARM_WITH_SWITCH instead)
CAUSE_UNKNOWN, CAUSE_HOLD, CAUSE_HELD_BACK, CAUSE_NOT_ARMED = 0, 1, 2, -1
CAUSE_WORDS = {CAUSE_UNKNOWN: "cause not known", CAUSE_HOLD: "the autopilot not in GUIDED",
               CAUSE_HELD_BACK: "drive commands held back", CAUSE_NOT_ARMED: "not armed"}


def commanding(v, w):
    """Does a command (m/s, rad/s) ask for motion?"""
    return abs(v) >= CMD_V_MPS or abs(w) >= CMD_W_RADS


def not_moving_cause(armed, mode, out_flowing):
    """The likeliest cause, first the ones the operator can fix: not armed, the autopilot not
    taking drive commands (mode), the commands not leaving twist_mux (a STOP lock), else unknown."""
    if not armed:
        return CAUSE_NOT_ARMED
    if mode != "GUIDED":
        return CAUSE_HOLD
    if not out_flowing:
        return CAUSE_HELD_BACK
    return CAUSE_UNKNOWN


def _moved(a, b):
    turn = abs((math.degrees(b[2] - a[2]) + 180.0) % 360.0 - 180.0)
    return math.hypot(b[0] - a[0], b[1] - a[1]) >= MOVED_M or turn >= TURNED_DEG


class MotionWatch:
    """step() is True once motion has been asked for `wait_s` without the rover moving."""

    def __init__(self, wait_s=NOT_MOVING_S):
        self.wait_s = wait_s
        self.reset()

    def reset(self):
        self.since, self.at = None, None   # when the ask began, and the pose then (x, y, yaw)

    def step(self, now, asked, pose):
        if not asked or pose is None:
            self.reset()
            return False
        if self.since is None or _moved(self.at, pose):
            self.since, self.at = now, pose    # moving: the wait starts again from here
            return False
        return now - self.since >= self.wait_s
