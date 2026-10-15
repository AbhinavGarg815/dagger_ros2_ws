"""Saved maps for the app (Step C1): the list from the map manager, and pictures over HTTP.

The map manager (dagger_bringup/map_manager) owns the files; the gateway only
translates /maps/list into the contract's MapList and serves each map's PNG at
GET /maps/<name>.png -- plain HTTP beside /api, so a 50 kB picture never sits in
front of a STOP on the command socket.
"""
import urllib.parse

import tornado.web

from . import api_pb2 as pb


def picture_path(m):
    """Where the app fetches a map's picture; changes whenever the picture does."""
    return f"/maps/{urllib.parse.quote(m.name)}.png?v={m.picture_version}"


def map_list_pb(msg):
    """dagger_msgs/MapList -> the contract's MapList (None: no list heard yet)."""
    out = pb.MapList()
    if msg is None:
        return out
    for m in msg.maps:
        corners = list(zip(m.corner_lat, m.corner_lon)) if m.placed else []
        out.maps.append(pb.MapSummary(
            name=m.name, width_m=m.width_m, height_m=m.height_m, placed=len(corners) == 4,
            corners=[pb.Waypoint(latitude_deg=la, longitude_deg=lo) for la, lo in corners]
            if len(corners) == 4 else [],
            placed_error_m=m.placed_error_m, picture_path=picture_path(m),
            picture_width_px=m.picture_width_px, picture_height_px=m.picture_height_px,
            created_at_ms=m.created_ms))
    out.shown = msg.shown if any(m.name == msg.shown for m in msg.maps) else ""
    return out


class MapPicture(tornado.web.RequestHandler):
    """GET /maps/<name>.png: the picture of a map in the latest list, else 404."""

    def initialize(self, gw):
        self.gw = gw

    def get(self, name):
        msg = self.gw.node.maps
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
