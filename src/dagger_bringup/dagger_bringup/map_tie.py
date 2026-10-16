"""Tie a map being made to the Earth, live (no ROS here). Step C2.

The same fit as tools/slam_georef.py, fed while the operator drives instead of
from a recording afterwards:

    p_enu = R(yaw) * p_map + t        (ENU metres about the datum lat/lon)

  * datum: the mean of the GPS fixes, each weighted 1/sigma^2 (fixes with sigma
    above MAX_SIGMA_M are left out: a drive that goes indoors carries fixes
    4-17 m off);
  * yaw: the circular mean of (dual-antenna heading - SLAM yaw) over the poses
    that have a heading within PAIR_S -- the heading is good to ~0.5 deg, the
    positions only to a few metres;
  * t: the weighted least-squares fit of the SLAM poses to the fixes.

Not placed until there are MIN_PAIRS heading pairs and position pairs: indoors,
or with no heading, the map is saved "not placed" (owner, 2026-10-15).
The result is written as <name>_geo.yaml, read by geo_map.GeoMap.
"""
import bisect
import collections
import math

import numpy as np

from .geo_map import enu_scale

MAX_SIGMA_M = 8.0
PAIR_S = 0.2           # a fix / heading counts for a pose this close in time
MIN_PAIRS = 10
KEEP_S = 3.0           # fixes and headings older than this wait for no pose


def to_latlon(fit, x, y):
    """map (x, y) -> (lat, lon), as geo_map.GeoMap.to_latlon does from the file."""
    c, s = math.cos(fit["rot"]), math.sin(fit["rot"])
    e, n = c * x - s * y + fit["tx"], s * x + c * y + fit["ty"]
    kx, ky = enu_scale(fit["lat0"])
    return fit["lat0"] + n / ky, fit["lon0"] + e / kx


def geo_yaml(fit, made_from):
    """The <name>_geo.yaml text, in tools/slam_georef.py's format."""
    return (f"# slam_map -> local ENU at the datum:  p_enu = R(yaw) * p_map + t\n"
            f"# made live by dagger_bringup/map_manager ({made_from})\n"
            f"datum_lat: {fit['lat0']:.9f}\ndatum_lon: {fit['lon0']:.9f}\n"
            f"yaw_deg: {math.degrees(fit['rot']):.4f}\ntx: {fit['tx']:.4f}\nty: {fit['ty']:.4f}\n"
            f"residual_rms_m: {fit['rms']:.3f}\nresidual_max_m: {fit['max']:.3f}\n")


class Tie:

    def __init__(self):
        self.fixes = []                          # (t, lat, lon, sigma), every usable fix
        self.recent_fixes = collections.deque()  # the last KEEP_S of them, for pairing
        self.recent_heads = collections.deque()  # (t, yaw_enu)
        self.dyaw = []                           # heading - SLAM yaw, per pose with a heading
        self.pairs = []                          # (x, y, the fix) per pose with a fix

    @staticmethod
    def _nearest(series, t):
        ts = [s[0] for s in series]
        i = bisect.bisect_left(ts, t)
        best = min((j for j in (i - 1, i) if 0 <= j < len(series)), key=lambda j: abs(ts[j] - t), default=None)
        return None if best is None or abs(ts[best] - t) > PAIR_S else series[best]

    @staticmethod
    def _trim(q, t):
        while q and q[0][0] < t - KEEP_S:
            q.popleft()

    def add_fix(self, t, lat, lon, sigma):
        if sigma > MAX_SIGMA_M:
            return
        f = (t, lat, lon, sigma)
        self.fixes.append(f)
        self.recent_fixes.append(f)
        self._trim(self.recent_fixes, t)

    def add_heading(self, t, yaw_enu):
        self.recent_heads.append((t, yaw_enu))
        self._trim(self.recent_heads, t)

    def add_pose(self, t, x, y, yaw):
        h = self._nearest(list(self.recent_heads), t)
        if h is not None:
            self.dyaw.append(math.remainder(h[1] - yaw, 2 * math.pi))
        g = self._nearest(list(self.recent_fixes), t)
        if g is not None:
            self.pairs.append((x, y, g))

    def fit(self):
        """dict(lat0, lon0, rot, tx, ty, rms, max), or None while not enough to place the map."""
        if len(self.dyaw) < MIN_PAIRS or len(self.pairs) < MIN_PAIRS:
            return None
        w = np.array([1 / f[3] ** 2 for f in self.fixes])
        lat0 = float((w * np.array([f[1] for f in self.fixes])).sum() / w.sum())
        lon0 = float((w * np.array([f[2] for f in self.fixes])).sum() / w.sum())
        kx, ky = enu_scale(lat0)
        rot = math.atan2(sum(math.sin(d) for d in self.dyaw), sum(math.cos(d) for d in self.dyaw))
        P = np.array([(x, y) for x, y, _ in self.pairs])
        Q = np.array([((g[2] - lon0) * kx, (g[1] - lat0) * ky) for _, _, g in self.pairs])
        W = np.array([1 / g[3] ** 2 for _, _, g in self.pairs])
        R = np.array([[math.cos(rot), -math.sin(rot)], [math.sin(rot), math.cos(rot)]])
        t = ((Q - P @ R.T) * W[:, None]).sum(axis=0) / W.sum()
        res = np.linalg.norm(P @ R.T + t - Q, axis=1)
        return dict(lat0=lat0, lon0=lon0, rot=rot, tx=float(t[0]), ty=float(t[1]),
                    rms=float(np.sqrt((res ** 2).mean())), max=float(res.max()))
