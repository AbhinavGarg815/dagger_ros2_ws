"""Saved routes, the recorder, and the route arithmetic. NO ROS, NO SOCKETS.

Routes are made in the app by driving (Phase 3, docs/proposals/phase3-routes.md
in DaggerGCS): while the operator drives with the sticks, the Recorder keeps a
point every `record_point_spacing_m` (and at sharp turns); Save puts them in the
RouteStore, one JSON file per route, so a route survives a gateway restart.
Missions also come planned on the app's map (SaveRoute) or from the autopilot,
as QGroundControl left it (ImportMission: mission_waypoints below).

Points are WGS-84 (lat, lon). Turning them into Nav2 goals is route_executor's
job (ROS); everything here is tested offline by test/test_routes.py.
"""
import json
import math
import os
import time
import uuid

from . import api_pb2 as pb

A, E2 = 6378137.0, 6.69437999014e-3  # WGS-84, as tools/gps_goal.py


def ll_to_en(lat, lon, lat0, lon0):
    """East/North metres of (lat, lon) relative to (lat0, lon0) (tools/gps_goal.py)."""
    p = math.radians(lat0)
    s = 1 - E2 * math.sin(p) ** 2
    m = A * (1 - E2) / s ** 1.5
    n = A / math.sqrt(s)
    return math.radians(lon - lon0) * n * math.cos(p), math.radians(lat - lat0) * m


def distance_m(a, b):
    e, n = ll_to_en(b[0], b[1], a[0], a[1])
    return math.hypot(e, n)


def path_length(points):
    return sum(distance_m(a, b) for a, b in zip(points, points[1:]))


# MAVLink mission items (MAV_CMD / MAV_FRAME) that ImportMission keeps: a
# waypoint with a position. Speed changes, loiters, return-to-launch and the
# rest are left out; Nav2 drives between the positions as for any route.
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_SPLINE_WAYPOINT = 82
POSITION_COMMANDS = (MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_SPLINE_WAYPOINT)
GLOBAL_FRAMES = (0, 3, 5, 6, 10, 11)   # GLOBAL, GLOBAL_RELATIVE_ALT, their _INT and _TERRAIN forms


def mission_waypoints(items):
    """The positions to drive, in order, from the autopilot's mission.

    `items` are (frame, command, lat, lon) in the order of MAVROS'
    /mavros/mission/waypoints. ArduPilot keeps its home point as item 0: left
    out. So is everything that is not a waypoint with a position. The fake
    rover in DaggerGCS uses the same rule.
    """
    return [(lat, lon) for frame, command, lat, lon in list(items)[1:]
            if command in POSITION_COMMANDS and frame in GLOBAL_FRAMES
            and math.isfinite(lat) and math.isfinite(lon) and (lat, lon) != (0.0, 0.0)]


def wall_ms():
    return int(time.time() * 1000)


class Route:
    """A saved route: points (lat, lon) in driving order."""

    def __init__(self, rid, name, points, created_ms):
        self.id, self.name, self.points, self.created_ms = rid, name, list(points), created_ms

    def summary(self):
        r = pb.RouteSummary(id=self.id, name=self.name, length_m=round(path_length(self.points), 2),
                            created_at_ms=self.created_ms)
        for lat, lon in self.points:
            r.waypoints.add(latitude_deg=lat, longitude_deg=lon)
        return r

    def to_json(self):
        return {"id": self.id, "name": self.name, "created_ms": self.created_ms,
                "points": [[lat, lon] for lat, lon in self.points]}

    @staticmethod
    def from_json(d):
        return Route(d["id"], d["name"], [tuple(p) for p in d["points"]], int(d["created_ms"]))


class RouteStore:
    """The saved routes, one JSON file each in `folder`. Writes are atomic
    (temp file + rename), so a power cut never leaves half a route."""

    def __init__(self, folder):
        self.folder = os.path.expanduser(folder)
        os.makedirs(self.folder, exist_ok=True)
        self.routes = {}
        for f in sorted(os.listdir(self.folder)):
            if f.endswith(".json"):
                try:
                    with open(os.path.join(self.folder, f), encoding="utf-8") as fh:
                        r = Route.from_json(json.load(fh))
                    self.routes[r.id] = r
                except (OSError, ValueError, KeyError):
                    pass                     # a damaged file is skipped, never fatal

    def all(self):
        return sorted(self.routes.values(), key=lambda r: (r.created_ms, r.id))

    def get(self, rid):
        return self.routes.get(rid)

    def name_taken(self, name):
        return any(r.name == name for r in self.routes.values())

    def suggested_name(self):
        n = len(self.routes) + 1
        while self.name_taken(f"Route {n}"):
            n += 1
        return f"Route {n}"

    def save(self, name, points):
        r = Route(f"r-{uuid.uuid4().hex[:8]}", name, points, wall_ms())
        path = os.path.join(self.folder, r.id + ".json")
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(r.to_json(), fh)
        os.replace(tmp, path)
        self.routes[r.id] = r
        return r


class Recorder:
    """Keeps a point every `spacing_m` of driving, and at sharp turns (a turn of
    more than TURN_DEG once at least TURN_MIN_M from the last point)."""

    TURN_DEG = 30.0
    TURN_MIN_M = 0.5

    def __init__(self, spacing_m):
        self.spacing_m = spacing_m
        self.points = []
        self.here = None

    def add(self, lat, lon):
        p = (lat, lon)
        self.here = p
        if not self.points:
            self.points.append(p)
            return
        d = distance_m(self.points[-1], p)
        if d >= self.spacing_m or (d >= self.TURN_MIN_M and len(self.points) >= 2
                                   and self._turn_deg(p) > self.TURN_DEG):
            self.points.append(p)

    def _turn_deg(self, p):
        a, b = self.points[-2], self.points[-1]
        e1, n1 = ll_to_en(b[0], b[1], a[0], a[1])
        e2, n2 = ll_to_en(p[0], p[1], b[0], b[1])
        d = math.degrees(math.atan2(e2, n2) - math.atan2(e1, n1))
        return abs((d + 180.0) % 360.0 - 180.0)

    def length_m(self):
        pts = self.points + ([self.here] if self.here and self.here != self.points[-1] else [])
        return path_length(pts) if len(pts) >= 2 else 0.0


def progress(points, target, here, start_total_m):
    """(fraction 0..1, metres remaining) with the rover at `here` driving to
    points[target]. `start_total_m`: the whole drive as it stood at the start
    (rover to the first point, plus the route)."""
    if here is None or not points:
        return 0.0, 0.0
    left = distance_m(here, points[target]) + path_length(points[target:])
    frac = 1.0 - left / start_total_m if start_total_m > 0 else 0.0
    return max(0.0, min(1.0, frac)), left


def window(points, start, here, max_m):
    """Indices points[start:end] to send to Nav2 now: consecutive points from
    `start` while each is within `max_m` of the rover — the global costmap is a
    rolling window (40 m: ~18 m to a goal, tools/gps_goal.py). Always at least
    one point, so a far first point is still tried (Nav2 says if it cannot)."""
    end = start + 1
    while end < len(points) and distance_m(here, points[end]) <= max_m:
        end += 1
    return start, end
