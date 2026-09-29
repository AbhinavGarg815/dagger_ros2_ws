"""lat/lon <-> SLAM map, through a map's <name>_geo.yaml (tools/slam_georef.py).

    p_enu = R(yaw) * p_map + t        (ENU metres about the datum lat/lon)

Shared by slam_geo_init (GNSS seed) and tools/gps_goal.py (lat/lon goals in the
SLAM map) so the two can never disagree about where a coordinate lands.
"""
import math

import yaml

A_WGS84 = 6378137.0
E2_WGS84 = 6.69437999014e-3


def enu_scale(lat0_deg):
    """Metres per degree (east, north) at lat0 -- fine for maps under a few km."""
    lat = math.radians(lat0_deg)
    s = 1 - E2_WGS84 * math.sin(lat) ** 2
    n = A_WGS84 / math.sqrt(s)
    m = A_WGS84 * (1 - E2_WGS84) / s ** 1.5
    return math.radians(1) * n * math.cos(lat), math.radians(1) * m


class GeoMap:

    def __init__(self, geo_file):
        with open(geo_file) as f:
            geo = yaml.safe_load(f)
        self.path = geo_file
        self.lat0, self.lon0 = float(geo["datum_lat"]), float(geo["datum_lon"])
        self.yaw_deg = float(geo["yaw_deg"])
        self.rot = math.radians(self.yaw_deg)
        self.tx, self.ty = float(geo["tx"]), float(geo["ty"])
        self.residual_rms = geo.get("residual_rms_m")
        self.kx, self.ky = enu_scale(self.lat0)

    def enu_to_map_vec(self, e, n):
        """Rotate an ENU vector (metres) into map axes (no translation)."""
        c, s = math.cos(self.rot), math.sin(self.rot)
        return c * e + s * n, -s * e + c * n

    def to_map(self, lat, lon, yaw_enu=0.0):
        """lat/lon (+ ENU heading) -> map (x, y, yaw)."""
        e = (lon - self.lon0) * self.kx - self.tx
        n = (lat - self.lat0) * self.ky - self.ty
        x, y = self.enu_to_map_vec(e, n)
        return x, y, math.remainder(yaw_enu - self.rot, 2 * math.pi)

    def to_latlon(self, x, y):
        """map (x, y) -> (lat, lon); inverse of to_map."""
        c, s = math.cos(self.rot), math.sin(self.rot)
        e, n = c * x - s * y + self.tx, s * x + c * y + self.ty
        return self.lat0 + n / self.ky, self.lon0 + e / self.kx

    def en_between(self, lat, lon, lat_ref, lon_ref):
        """ENU metres of (lat, lon) relative to (lat_ref, lon_ref)."""
        return (lon - lon_ref) * self.kx, (lat - lat_ref) * self.ky
