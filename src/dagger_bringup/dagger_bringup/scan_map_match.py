"""Brute-force 2D scan-to-map matching against a saved occupancy map (pgm + yaml).

Used to seed SLAM localization (slam_geo_init) and to check that it stays
locked. Score = mean distance (m, clipped at `clip`) from each scan point to the
nearest occupied cell -- ~0.05-0.10 m when a scan sits on the right place in the
map, ~0.7-1.0 m when it does not (lane2 on lane3fix, 2026-09-26).
Scan points must already be in the robot base frame (our /scan is in
base_footprint).
"""
import math

import cv2
import numpy as np
import yaml


class MapMatcher:

    def __init__(self, map_yaml, clip=1.0):
        with open(map_yaml) as f:
            meta = yaml.safe_load(f)
        path = meta["image"]
        if not path.startswith("/"):
            path = map_yaml.rsplit("/", 1)[0] + "/" + path
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"cannot read map image {path}")
        occ_thresh = meta.get("occupied_thresh", 0.65)
        occupied = img < int(round((1.0 - occ_thresh) * 255))
        self.occupied_count = int(occupied.sum())
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        self.h, self.w = img.shape
        self.clip = clip
        self.dist = np.minimum(
            cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5) * self.res, clip)

    def _lookup(self, wx, wy):
        u = ((wx - self.ox) / self.res).astype(np.int32)
        v = (self.h - 1) - ((wy - self.oy) / self.res).astype(np.int32)
        ok = (u >= 0) & (u < self.w) & (v >= 0) & (v < self.h)
        d = np.full(wx.shape, self.clip, dtype=np.float32)
        d[ok] = self.dist[v[ok], u[ok]]
        return d

    def score(self, pts, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return float(self._lookup(x + c * pts[:, 0] - s * pts[:, 1],
                                  y + s * pts[:, 0] + c * pts[:, 1]).mean())

    def _grid(self, pts, x0, y0, yaw0, radius, yaw_window, step, yaw_step):
        """Every (x, y) on a grid around (x0, y0), for each yaw in the window: the grid
        (gx, gy) and per yaw (best score, x, y, yaw, all scores)."""
        xs = np.arange(x0 - radius, x0 + radius + 1e-6, step, dtype=np.float32)
        ys = np.arange(y0 - radius, y0 + radius + 1e-6, step, dtype=np.float32)
        gx, gy = np.meshgrid(xs, ys)
        gx, gy = gx.ravel()[:, None], gy.ravel()[:, None]
        results = []
        for yaw in np.arange(yaw0 - yaw_window, yaw0 + yaw_window + 1e-9, yaw_step):
            c, s = math.cos(yaw), math.sin(yaw)
            rx = c * pts[:, 0] - s * pts[:, 1]
            ry = s * pts[:, 0] + c * pts[:, 1]
            sc = self._lookup(gx + rx[None, :], gy + ry[None, :]).mean(axis=1)
            i = int(np.argmin(sc))
            results.append((float(sc[i]), float(gx[i, 0]), float(gy[i, 0]), float(yaw), sc))
        return gx, gy, results

    def _refine(self, pts, best):
        sc, x, y, yaw = best[:4]
        for dyaw in np.radians(np.arange(-2.0, 2.01, 0.25)):
            for dx in np.arange(-0.2, 0.201, 0.05):
                for dy in np.arange(-0.2, 0.201, 0.05):
                    s2 = self.score(pts, x + dx, y + dy, best[3] + dyaw)
                    if s2 < sc:
                        sc, x, y, yaw = s2, x + dx, y + dy, best[3] + dyaw
        return sc, x, y, yaw

    def search(self, pts, x0, y0, yaw0, radius, yaw_window, step=0.2, yaw_step=math.radians(2)):
        """Best (score, x, y, yaw) on a grid around (x0, y0, yaw0), plus the best
        score found farther than 1.5 m from it (for ambiguity), then refined."""
        gx, gy, results = self._grid(pts, x0, y0, yaw0, radius, yaw_window, step, yaw_step)
        best = min(results, key=lambda r: r[0])
        # second best: anything more than 1.5 m away from the best, any yaw
        far = (np.hypot(gx[:, 0] - best[1], gy[:, 0] - best[2]) > 1.5)
        second = min(float(r[4][far].min()) if far.any() else self.clip for r in results)
        sc, x, y, yaw = self._refine(pts, best)
        return sc, x, y, yaw, second

    def search_here(self, pts, x0, y0, radius, yaw0=0.0, yaw_window=math.pi, turned_deg=30.0,
                    step=0.2, yaw_step=math.radians(2)):
        """"I am here" (Step D2): the operator's tap, and with no direction every way round
        (yaw_window pi). (score, x, y, yaw, second, turned): `second` as in search (a
        different place fits as well), `turned` the best score at about the same place
        (within 1.5 m) facing more than turned_deg another way -- a room that looks the
        same both ways round, where only the operator can say which way the rover faces."""
        gx, gy, results = self._grid(pts, x0, y0, yaw0, radius, yaw_window, step, yaw_step)
        best = min(results, key=lambda r: r[0])
        d = np.hypot(gx[:, 0] - best[1], gy[:, 0] - best[2])
        far, near = d > 1.5, d <= 1.5
        second = min(float(r[4][far].min()) if far.any() else self.clip for r in results)
        apart = lambda a, b: abs((a - b + math.pi) % (2 * math.pi) - math.pi)  # noqa: E731
        turned = min((float(r[4][near].min()) for r in results
                      if apart(r[3], best[3]) > math.radians(turned_deg)), default=self.clip)
        sc, x, y, yaw = self._refine(pts, best)
        return sc, x, y, yaw, second, turned


FOUND, NOT_FOUND, NEED_FRONT = "found", "not found", "need front"


def here_verdict(score, second, turned, has_front, accept=0.15, ambig=2.0, floor=0.05):
    """"I am here" (Step D2): what search_here's numbers mean. Not found when the best fit
    is poor or another place fits nearly as well; with no direction given, the front is
    asked for when another way round at the same place fits nearly as well. "Nearly as
    well" is against at least `floor` (a locked scan scores ~0.05 m): a near-perfect fit
    must not make every other fit look clearly worse."""
    ref = ambig * max(score, floor)
    if score > accept or second < ref:
        return NOT_FOUND
    if not has_front and turned < ref:
        return NEED_FRONT
    return FOUND


def scan_points(msg, max_range=20.0, max_points=300):
    """LaserScan -> Nx2 float32 points in the scan frame, evenly thinned."""
    r = np.asarray(msg.ranges, dtype=np.float32)
    a = msg.angle_min + np.arange(len(r), dtype=np.float32) * msg.angle_increment
    ok = np.isfinite(r) & (r > msg.range_min) & (r < min(msg.range_max, max_range))
    pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
    if len(pts) > max_points:
        pts = pts[np.linspace(0, len(pts) - 1, max_points).astype(int)]
    return pts
