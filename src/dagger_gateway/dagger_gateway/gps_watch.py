"""Is GPS good enough to drive by? Step D3 (GPS lost during a job). NO ROS.

DaggerGCS docs/proposals/operating-modes.md section 4, with the owner's answers of
2026-10-19: GPS is LOST when, for `lost_s` without a break, there is no 3-D fix, or
the receiver's accuracy is worse than `accuracy_m`, or the dual-antenna heading is
gone (the Cube has no other heading: EK3_SRC1_YAW 2). It is BACK after `back_s` of
good samples without a break -- and the gateway still switches back only when parked.
Losing only RTK (fixed -> float or plain 3-D) is not "lost": RtkWatch says so, and
the job goes on.

Disagreement: the autopilot's own heading or position against the receiver's, for
`lost_s` -- e.g. the Cube on its LiDAR source set (RC10 middle) outdoors, when its
lat/lon is not tied to the Earth (2026-10-19: 129 deg and 5.7 m off).
"""
import math


class Hold:
    """True once `cond` has held for `after_s` without a break; false at once when it ends."""

    def __init__(self, after_s):
        self.after_s, self.since = after_s, None

    def feed(self, now, cond):
        if not cond:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.after_s


class GpsWatch:

    def __init__(self, lost_s=5.0, accuracy_m=3.0, back_s=10.0):
        self.accuracy_m = accuracy_m
        self.bad = Hold(lost_s)
        self.good = Hold(back_s)
        self.lost = False
        self.bad_since = None          # monotonic s: when the bad samples began (for the snapshot)

    def good_sample(self, fix_type, h_acc_m, heading_ok):
        return (fix_type is not None and fix_type >= 3 and h_acc_m is not None
                and 0 < h_acc_m <= self.accuracy_m and bool(heading_ok))

    def feed(self, now, fix_type, h_acc_m, heading_ok):
        """One receiver sample; returns "lost", "back" or None (no change)."""
        ok = self.good_sample(fix_type, h_acc_m, heading_ok)
        if not ok and self.bad.since is None:
            self.bad_since = now
        long_bad, long_good = self.bad.feed(now, not ok), self.good.feed(now, ok)
        if not self.lost and long_bad:
            self.lost = True
            return "lost"
        if self.lost and long_good:
            self.lost = False
            return "back"
        return None


class RtkWatch:
    """RTK lost: the base is sending (heard, fixed) but the rover is not RTK float or
    fixed (GPS_RAW_INT fix_type 5/6) for `after_s`."""

    def __init__(self, after_s=5.0):
        self.hold = Hold(after_s)

    def feed(self, now, base_sending, fix_type):
        return self.hold.feed(now, bool(base_sending) and fix_type is not None and fix_type < 5)


def angle_diff_deg(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


class DisagreeWatch:
    """The autopilot's heading / position against the receiver's, while GPS is good."""

    def __init__(self, after_s=5.0, max_deg=20.0, max_m=3.0):
        self.hold = Hold(after_s)
        self.max_deg, self.max_m = max_deg, max_m
        self.deg, self.m = 0.0, 0.0

    def feed(self, now, gps_good, fcu_heading_deg, gnss_heading_deg, apart_m):
        """apart_m: metres between the autopilot's lat/lon and the receiver's (None: unknown)."""
        self.deg = (angle_diff_deg(fcu_heading_deg, gnss_heading_deg)
                    if fcu_heading_deg is not None and gnss_heading_deg is not None else 0.0)
        self.m = apart_m if apart_m is not None and math.isfinite(apart_m) else 0.0
        return self.hold.feed(now, gps_good and (self.deg > self.max_deg or self.m > self.max_m))
