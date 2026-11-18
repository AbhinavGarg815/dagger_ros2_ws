"""Saved maps for the app (Step C1): the list from the map manager, and pictures over HTTP.

The map manager (dagger_bringup/map_manager) owns the files; the gateway only
translates /maps/list into the contract's MapList and serves each map's PNG at
GET /maps/<name>.png -- plain HTTP beside /api, so a 50 kB picture never sits in
front of a STOP on the command socket.
"""
import math
import urllib.parse

import tornado.web

from . import api_pb2 as pb


LIVE = "~live"          # the map being made, as its picture is named
# The map of this drive (CAPABILITY_DRIVE_MAP, owner 2026-11-17): the map manager's shadow mapping
# whenever no saved map is shown and none is being made, listed under this name. The rover is
# always "found" on it -- it is making it.
DRIVE = "~drive"


A_WGS84, E2_WGS84 = 6378137.0, 6.69437999014e-3


class SlamHeard:
    """What the shadow localization last said about the map in use: /slam/localized and
    /slam/finding. Both are latched, so after a gateway restart they arrive once, in any order
    with /maps/list. Kept as heard and judged against the map in use, they survive that order;
    a CHANGE of map drops them (the new localization says for itself). Before 2026-10-23 the
    first /maps/list cleared them as if the map had changed: after every gateway restart with
    a map found, *Place on the map* stayed red until SLAM's verdict next changed (owner)."""

    def __init__(self):
        self.map_known = False
        self.localizing = ""
        self.localized_heard = None
        self.finding_heard = None

    def map_in_use(self, name):
        """From /maps/list. True when the map in use changed (its verdicts are dropped)."""
        changed = self.map_known and name != self.localizing
        if changed:
            self.localized_heard = self.finding_heard = None
        self.map_known, self.localizing = True, name
        return changed

    @property
    def localized(self):
        """Found on the map in use (always on the map of this drive: it is being made)."""
        return bool(self.localizing) and (self.localizing == DRIVE or bool(self.localized_heard))

    @property
    def finding(self):
        """slam_geo_init's step (the contract's MapFinding values), or None."""
        if not self.localizing:
            return None
        return pb.MAP_FINDING_FOUND if self.localizing == DRIVE else self.finding_heard


def _enu_scale(lat0_deg):
    """Metres per degree (east, north) at lat0: as dagger_bringup/geo_map.enu_scale."""
    lat = math.radians(lat0_deg)
    s = 1 - E2_WGS84 * math.sin(lat) ** 2
    n = A_WGS84 / math.sqrt(s)
    m = A_WGS84 * (1 - E2_WGS84) / s ** 1.5
    return math.radians(1) * n * math.cos(lat), math.radians(1) * m


def to_map(geo, lat, lon):
    """lat/lon -> (x, y) on a placed map, exactly as geo_map.GeoMap.to_map (and so as
    the map's picture was placed): geo = MapInfo.geo (lat0, lon0, yaw_rad, tx, ty)."""
    lat0, lon0, rot, tx, ty = geo
    kx, ky = _enu_scale(lat0)
    e, n = (lon - lon0) * kx - tx, (lat - lat0) * ky - ty
    c, s = math.cos(rot), math.sin(rot)
    return c * e + s * n, -s * e + c * n


def to_latlon(geo, x, y):
    """(x, y) on a placed map -> lat/lon; the inverse of to_map."""
    lat0, lon0, rot, tx, ty = geo
    kx, ky = _enu_scale(lat0)
    c, s = math.cos(rot), math.sin(rot)
    e, n = c * x - s * y + tx, s * x + c * y + ty
    return lat0 + n / ky, lon0 + e / kx


def map_info(msg, name):
    """The MapInfo of any map in the list (placed or not), or None."""
    if msg is None:
        return None
    return next((m for m in msg.maps if m.name == name), None)


# Step D2: points ON a map's picture (what the app shows and the operator taps) -- metres
# from its bottom-left corner, x right, y up -- and the map's own frame differ only by
# the picture's origin (its .yaml origin; map_saver never turns the picture).
def to_picture(m, x, y):
    """(x, y) on the map -> metres on its picture."""
    return x - m.origin_x, y - m.origin_y


def from_picture(m, px, py):
    """Metres on a map's picture -> (x, y) on the map."""
    return px + m.origin_x, py + m.origin_y


def picture_heading(yaw):
    """A yaw on the map (rad, anticlockwise from its x axis) -> degrees clockwise from the
    picture's up, as the app draws it."""
    return (90.0 - math.degrees(yaw)) % 360.0


def placed_map(msg, name):
    """The MapInfo of a placed map in the list (with its tie), or None."""
    if msg is None:
        return None
    return next((m for m in msg.maps if m.name == name and m.placed and len(m.geo) == 5), None)


def place_on_map(msg, found, lat, lon):
    """A place on the Earth onto the map the rover is found on, when that map is placed:
    (MapInfo, (x, y)) -- else None. found: the gateway's found_pose (map name, x, y, yaw) or None.
    Go to a place with no GPS (owner, 2026-11-18): the app sends a tap on a placed map as lat/lon."""
    if found is None:
        return None
    m = placed_map(msg, found[0])
    return (m, to_map(tuple(m.geo), lat, lon)) if m is not None else None


def picture_path(m):
    """Where the app fetches a map's picture; changes whenever the picture does."""
    return f"/maps/{urllib.parse.quote(m.name or LIVE)}.png?v={m.picture_version}"


def summary_pb(m):
    """dagger_msgs/MapInfo -> the contract's MapSummary."""
    corners = list(zip(m.corner_lat, m.corner_lon)) if m.placed else []
    placed = len(corners) == 4
    return pb.MapSummary(
        name=m.name, width_m=m.width_m, height_m=m.height_m, placed=placed,
        corners=[pb.Waypoint(latitude_deg=la, longitude_deg=lo) for la, lo in corners] if placed else [],
        placed_error_m=m.placed_error_m, picture_path=picture_path(m),
        picture_width_px=m.picture_width_px, picture_height_px=m.picture_height_px,
        created_at_ms=m.created_ms, drive_map=m.name == DRIVE)


def mapping_pb(status):
    """dagger_msgs/MappingStatus -> the contract's MappingProgress, or None when not mapping."""
    if status is None or not status.active:
        return None
    out = pb.MappingProgress(suggested_name=status.suggested_name, saving=status.saving)
    if status.live.picture:
        out.map.CopyFrom(summary_pb(status.live))
    return out


def map_list_pb(msg, drive_in_use=False):
    """dagger_msgs/MapList -> the contract's MapList (None: no list heard yet). The map of this
    drive is listed whenever the rover makes it (drive_map, so it can be saved: SaveMap), and shown
    only while it is the map in use (drive_in_use: GPS not good enough to drive on)."""
    out = pb.MapList()
    if msg is None:
        return out
    for m in msg.maps:
        out.maps.append(summary_pb(m))
    out.shown = msg.shown if any(m.name == msg.shown for m in msg.maps) else ""
    if drive_in_use and any(m.name == DRIVE for m in msg.maps):
        out.shown = DRIVE
    return out


_pictures = {}          # picture file -> its alpha (PIL image), for seen_on_picture


def seen_on_picture(m, x, y):
    """(x, y) on the map is where the rover has seen (Go to a place, owner 2026-11-17): on the
    picture and not transparent -- transparent is never seen. Off the picture: not seen."""
    if not m.picture or m.picture_width_px == 0 or m.picture_height_px == 0 or m.width_m <= 0:
        return False
    px, py = to_picture(m, x, y)
    res = m.width_m / m.picture_width_px
    col, row = int(px / res), int(m.picture_height_px - 1 - py / res)
    if not (0 <= col < m.picture_width_px and 0 <= row < m.picture_height_px):
        return False
    alpha = _pictures.get(m.picture)
    if alpha is None:
        from PIL import Image
        try:
            with Image.open(m.picture) as img:
                alpha = img.convert("LA").getchannel("A")
        except OSError:
            return False
        _pictures.clear()               # one picture at a time is enough: the map in use
        _pictures[m.picture] = alpha
    return alpha.getpixel((col, row)) > 0


class MapPicture(tornado.web.RequestHandler):
    """GET /maps/<name>.png: the picture of a map in the latest list, else 404."""

    def initialize(self, gw):
        self.gw = gw

    def get(self, name):
        msg, live = self.gw.node.maps, self.gw.node.mapping
        if name == LIVE:
            found = live.live if live is not None and live.active and live.live.picture else None
        else:
            found = next((m for m in msg.maps if m.name == name), None) if msg is not None else None
        if found is None:
            raise tornado.web.HTTPError(404)
        try:
            with open(found.picture, "rb") as f:
                data = f.read()
        except OSError:
            raise tornado.web.HTTPError(404)
        self.set_header("Content-Type", "image/png")
        self.set_header("Cache-Control", "max-age=86400")   # the path changes with the picture
        self.write(data)
