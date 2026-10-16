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


# Return to Home (DaggerGCS docs/proposals/operating-modes.md, owner 2026-10-13):
# back along the rover's own trail.
TRAIL_STEP_M = 1.0          # the trail keeps a point every metre driven
HOME_POINT_SPACING_M = 3.0  # the way home: a point every 3 m of trail
LOOP_CUT_M = 2.0            # where the trail comes back this close, the loop is skipped


class Trail:
    """Where the rover has been since home, a point every TRAIL_STEP_M."""

    def __init__(self, home):
        self.home = tuple(home)
        self.points = [self.home]

    def add(self, lat, lon):
        if distance_m(self.points[-1], (lat, lon)) >= TRAIL_STEP_M:
            self.points.append((lat, lon))

    def reset(self):
        """Home again: the trail starts over."""
        self.points = [self.home]


def home_path(trail, here):
    """The way home: from `here` back along `trail` (oldest first, trail[0] = home),
    loops skipped, a point every HOME_POINT_SPACING_M, ending exactly at home."""
    back = [tuple(here)] + list(reversed(trail))
    out, i = [back[0]], 0
    while i < len(back) - 1:
        # jump to the last trail point that comes back near this one (a loop), else the next
        near = [k for k in range(i + 2, len(back)) if distance_m(back[k], back[i]) < LOOP_CUT_M]
        i = near[-1] if near else i + 1
        if i == len(back) - 1 or distance_m(back[i], out[-1]) >= HOME_POINT_SPACING_M:
            out.append(back[i])
    return out[1:] or [tuple(trail[0])]


# The waypoint check: OccupancyGrid values from a Nav2 costmap. 100 lethal, 99
# inscribed (the rover's footprint would touch something): Theta* refuses a goal
# there ("Either of the start or goal pose are an obstacle!", 2026-10-13).
BLOCKED_COST = 99
NUDGE_M = 1.0


def clear_spot(cost_at, cx, cy, radius_cells):
    """The nearest cell to (cx, cy), within radius_cells, whose cost is known and
    under BLOCKED_COST -- or None. cost_at(x, y) gives -1 outside the grid / unknown.
    (cx, cy) itself counts when clear."""
    c = cost_at(cx, cy)
    if 0 <= c < BLOCKED_COST or c < 0:
        return cx, cy
    best = None
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            d2 = dx * dx + dy * dy
            if d2 > radius_cells * radius_cells or (best is not None and d2 >= best[0]):
                continue
            v = cost_at(cx + dx, cy + dy)
            if 0 <= v < BLOCKED_COST:
                best = (d2, cx + dx, cy + dy)
    return None if best is None else (best[1], best[2])


def wall_ms():
    return int(time.time() * 1000)


def map_to_odom(slam, odom):
    """Where the map lies in odom, from the rover's pose in both at the same moment:
    slam = (x, y, yaw) on the map, odom = (x, y, yaw) in odom. Returns (tx, ty, th)
    with p_odom = R(th) * p_map + (tx, ty) -- a map mission's goals go to Nav2 in odom
    this way (Step D1), so neither the GPS error nor the GPS map frame moves them."""
    th = odom[2] - slam[2]
    c, s = math.cos(th), math.sin(th)
    return odom[0] - (c * slam[0] - s * slam[1]), odom[1] - (s * slam[0] + c * slam[1]), th


def apply(t, x, y):
    """p_odom for a map point, given map_to_odom's (tx, ty, th)."""
    c, s = math.cos(t[2]), math.sin(t[2])
    return t[0] + c * x - s * y, t[1] + s * x + c * y


class Route:
    """A saved route: points (lat, lon) in driving order. A map mission (Step D1)
    also has the map it belongs to and its points ON that map (x, y), which are
    what is driven; its lat/lon are for drawing and progress."""

    def __init__(self, rid, name, points, created_ms, map_name="", xy=None):
        self.id, self.name, self.points, self.created_ms = rid, name, list(points), created_ms
        self.map_name, self.xy = map_name, [tuple(p) for p in xy] if xy else None

    def summary(self):
        r = pb.RouteSummary(id=self.id, name=self.name, length_m=round(path_length(self.points), 2),
                            created_at_ms=self.created_ms, map_name=self.map_name)
        for lat, lon in self.points:
            r.waypoints.add(latitude_deg=lat, longitude_deg=lon)
        return r

    def to_json(self):
        d = {"id": self.id, "name": self.name, "created_ms": self.created_ms,
             "points": [[lat, lon] for lat, lon in self.points]}
        if self.map_name:
            d["map"], d["xy"] = self.map_name, [[x, y] for x, y in self.xy]
        return d

    @staticmethod
    def from_json(d):
        return Route(d["id"], d["name"], [tuple(p) for p in d["points"]], int(d["created_ms"]),
                     d.get("map", ""), d.get("xy"))


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

    def save(self, name, points, map_name="", xy=None):
        r = Route(f"r-{uuid.uuid4().hex[:8]}", name, points, wall_ms(), map_name, xy)
        path = os.path.join(self.folder, r.id + ".json")
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(r.to_json(), fh)
        os.replace(tmp, path)
        self.routes[r.id] = r
        return r

    def delete(self, rid):
        """Remove a route and its file for good. False when there is no such route."""
        if self.routes.pop(rid, None) is None:
            return False
        try:
            os.remove(os.path.join(self.folder, rid + ".json"))
        except FileNotFoundError:
            pass
        return True


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


def window(points, start, here, max_m, dist=None):
    """Indices points[start:end] to send to Nav2 now: consecutive points from
    `start` while each is within `max_m` of the rover — the global costmap is a
    rolling window (40 m: ~18 m to a goal, tools/gps_goal.py). Always at least
    one point, so a far first point is still tried (Nav2 says if it cannot)."""
    end = start + 1
    dist = dist or distance_m             # lat/lon; map missions pass math.dist (metres)
    while end < len(points) and dist(here, points[end]) <= max_m:
        end += 1
    return start, end
