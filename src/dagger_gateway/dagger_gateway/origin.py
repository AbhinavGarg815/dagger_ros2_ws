"""The Cube's EKF origin when the rover starts without GPS (owner, 2026-10-26).

ArduRover drops every SET_POSITION_TARGET_LOCAL_NED -- the GUIDED drive commands Nav2's
velocities become -- while its EKF has no origin (Rover 4.6.3, GCS_Mavlink.cpp: "need ekf
origin"). Outdoors the first GPS fix sets one. Booted indoors with no fix, nothing does: the
rover localises well on the map, Nav2 plans and commands, and the tracks stay at neutral.

So: the gateway asks the Cube for its origin (MAV_CMD_REQUEST_MESSAGE GPS_GLOBAL_ORIGIN) and
keeps every origin it hears (a file, across restarts). With none heard ORIGIN_WAIT_S after the
Cube is first heard, and no GPS fix, it sets the kept one (SET_GPS_GLOBAL_ORIGIN) -- or, never
having heard one, home's lat/lon at 0 m. Moves nothing. Where the origin is matters little: the
LiDAR position is relative, and GPS, when it comes, is fused relative to the origin.
"""
import json
import math
import os

ORIGIN_WAIT_S = 20.0       # after the Cube is first heard: GPS gets this long to set the origin itself
ORIGIN_ASK_S = 5.0         # how often the Cube is asked for its origin until it has one
ORIGIN_RESEND_S = 30.0     # set again if the Cube has still not reported an origin
ORIGIN_TRIES = 3


class OriginStore:
    """The last EKF origin heard from the Cube: one small JSON file. Missing or damaged: None."""

    def __init__(self, path):
        self.path = os.path.expanduser(path)

    def load(self):
        try:
            with open(self.path, encoding="utf-8") as f:
                d = json.load(f)
            o = (float(d["lat"]), float(d["lon"]), float(d["alt"]))
        except (OSError, ValueError, KeyError, TypeError):
            return None
        return o if all(math.isfinite(v) for v in o) and o[:2] != (0.0, 0.0) else None

    def save(self, lat, lon, alt):
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"lat": lat, "lon": lon, "alt": alt}, f)
        os.replace(tmp, self.path)


class OriginKeeper:
    """What to do about the Cube's origin, called often: None, "ask", or ("set", (lat, lon, alt))."""

    def __init__(self, wait_s=ORIGIN_WAIT_S, ask_s=ORIGIN_ASK_S, resend_s=ORIGIN_RESEND_S, tries=ORIGIN_TRIES):
        self.wait_s, self.ask_s, self.resend_s, self.tries = wait_s, ask_s, resend_s, tries
        self._reset()

    def _reset(self):
        self.first_heard = None            # monotonic s the Cube was first heard
        self.asked_at = None
        self.sent_at, self.sent = None, 0

    def step(self, now, fcu_ok, origin_known, gps_fix, fallback):
        if not fcu_ok:
            self._reset()                  # a Cube that restarts has lost its origin
            return None
        if origin_known:
            return None
        if self.first_heard is None:
            self.first_heard = now
        if (not gps_fix and fallback is not None and now - self.first_heard >= self.wait_s
                and self.sent < self.tries and (self.sent_at is None or now - self.sent_at >= self.resend_s)):
            self.sent, self.sent_at = self.sent + 1, now
            return ("set", fallback)
        if self.asked_at is None or now - self.asked_at >= self.ask_s:
            self.asked_at = now
            return "ask"
        return None
