"""ArUco-marker tracking for click-to-select following -- no ROS in here.

The marker counterpart of person_tracking.py: same states, same target and
search-hint interface, so follow_controller, the collision monitor and the
browser UI are shared. Node wrapper: marker_tracker.py. Unit tests:
tools/test_marker_tracking.py.

A marker's ID IS its identity, so re-identification after a loss is exact: the
same ID seen again. What needs care is TRUE POSITIVES:
  * only allowed IDs (the markers you own) are tracked;
  * a marker is CONFIRMED -- shown as selectable -- after min_hits sightings
    within confirm_window s, so a one-frame false decode never becomes a target;
  * a sighting counts only with plausible geometry: side >= min_side_px, PnP
    reprojection error <= max_reproj_px, facing the camera within max_tilt_deg,
    range within [min_range, max_range];
  * while tracking, a sighting outside the motion gate is ignored (a second
    print of the same ID elsewhere) unless it persists for jump_frames frames;
  * after a loss, following resumes only after resume_hits sightings.

Range: from the marker's printed size (PnP, IPPE_SQUARE), or -- where the LiDAR
has a return at the marker's bearing that agrees with it (lidar_tol) -- the
LiDAR's, which stays accurate far away where the marker is a few pixels. With
size 0 (not set) only a LiDAR range counts, unchecked.

No LiDAR hold (v1): the carrier may be a car, and a car-sized return is exactly
what the person hold refuses as a wall. Out of view = lost -> SEARCHING -> the
same ID seen again resumes.
"""
import math

import numpy as np

from dagger_bringup.person_tracking import LOST, NONE, SEARCHING, TRACKING, range_at, to_odom

NAN = float("nan")


def aruco_detector(dictionary):
    """'DICT_4X4_50' etc. -> a detect(gray) function returning [(marker_id, corners
    as 8 floats)], sub-pixel corners in ArUco order. OpenCV 4.5.4 (Jammy) API."""
    import cv2
    A = cv2.aruco
    if not hasattr(A, dictionary) or not dictionary.startswith("DICT_"):
        raise ValueError(f"unknown ArUco dictionary {dictionary!r} (e.g. DICT_4X4_50, "
                         f"DICT_5X5_100, DICT_6X6_250, DICT_ARUCO_ORIGINAL)")
    d = A.getPredefinedDictionary(getattr(A, dictionary))
    p = A.DetectorParameters_create()
    # Sub-pixel corners (the range comes from them), but NOT OpenCV's built-in
    # refinement: its fixed 5 px window is wider than a far marker's cells and
    # pulls the corners outward -- range 3-7 % short at 3-5 m for a 15 cm marker
    # (rendered through this rover's camera, 2026-09-30). Window = half a cell,
    # 1..5 px: +0.2 % at 1.2 m, +1 % at 3 m, +2 % at 5 m.
    p.cornerRefinementMethod = A.CORNER_REFINE_NONE
    cells = int(d.markerSize) + 2                           # bits + the black border
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

    def detect(gray):
        corners, ids, _ = A.detectMarkers(gray, d, parameters=p)
        if ids is None:
            return []
        out = []
        for i, c in zip(ids.ravel(), corners):
            c = c.reshape(4, 2).astype(np.float32)
            side = float(np.mean(np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1)))
            w = int(max(1, min(5, round(side / cells * 0.5))))
            c = cv2.cornerSubPix(gray, c.reshape(-1, 1, 2).copy(), (w, w), (-1, -1), crit)
            out.append((int(i), [float(v) for v in c.reshape(8)]))
        return out
    return detect


def quat_to_R(q):
    """(x, y, z, w) -> 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def marker_pose(corners, size, K, dist):
    """Marker pose in the camera OPTICAL frame from its 4 corners (ArUco order:
    top-left, top-right, bottom-right, bottom-left as printed) and printed side
    length `size` (m). -> (t (3,), R (3, 3), reprojection error px, sure) or None.
    R[:, 2] is the marker's normal, pointing out of the printed face.

    A flat square has TWO poses that fit (tilted one way or the other); seen
    head-on and small they fit equally well, and the rotation of the better one
    is noise (a marker facing the camera at 3 m came out turned 25 deg). `sure`
    is True only when the second pose fits clearly worse. The POSITION is nearly
    the same in both, so range and bearing do not depend on it."""
    import cv2
    h = size / 2.0
    obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    img = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    fx, fy, cx, cy = K
    Km = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    d = np.asarray(dist if dist is not None else [], dtype=np.float64)
    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, Km, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not n:
        return None
    errs = []
    for rvec, tvec in zip(rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rvec, tvec, Km, d)
        errs.append(float(np.mean(np.linalg.norm(proj.reshape(4, 2) - img, axis=1))))
    k = int(np.argmin(errs))
    sure = len(errs) < 2 or max(errs) > 3.0 * errs[k] + 0.3
    R, _ = cv2.Rodrigues(rvecs[k])
    return np.asarray(tvecs[k]).reshape(3), R, errs[k], sure


class MarkerObs:
    """One marker sighting, located in base_footprint (MarkerTracker.observe)."""
    __slots__ = ("marker_id", "corners", "xy", "bearing", "heading", "side_px", "source", "reject")

    def __init__(self, marker_id, corners, xy=None, bearing=NAN, heading=NAN, side_px=0.0,
                 source="", reject=None):
        self.marker_id, self.corners = int(marker_id), [float(v) for v in corners]
        self.xy, self.bearing, self.heading = xy, bearing, heading
        self.side_px, self.source, self.reject = side_px, source, reject


class MarkerTrack:
    def __init__(self, marker_id, tid, now):
        self.marker_id, self.id = marker_id, tid
        self.born = now
        self.sightings = []            # times of recent accepted sightings (confirmation)
        self.confirmed = False
        self.t_seen = None
        self.visible = False
        self.corners = None
        self.pos, self.vel, self.t_pos = None, (0.0, 0.0), None    # odom
        self.range = self.bearing = self.heading = NAN
        self.source = ""
        self.jumps = 0                 # consecutive sightings outside the motion gate
        self.ever_ranged = False       # only ranged markers can be selected
        self.xy = None                 # this frame's base-frame position, if visible + ranged

    def predicted_pos(self, now):
        if self.pos is None:
            return None
        dt = min(now - self.t_pos, 3.0)
        return self.pos[0] + self.vel[0] * dt, self.pos[1] + self.vel[1] * dt


class MarkerTracker:

    def __init__(self, id_base=1_000_000, allowed_ids=None, min_hits=3, confirm_window=1.0,
                 min_side_px=10.0, max_reproj_px=2.0, max_tilt_deg=75.0, min_range=0.3,
                 max_range=15.0, lidar_tol=0.3, lidar_tol_rel=0.1, max_speed=3.0,
                 track_gate=1.0, jump_frames=3, lost_after=0.5, reid_window=30.0,
                 resume_hits=2, forget_after=10.0):
        self.id_base = id_base
        self.allowed = set(allowed_ids) if allowed_ids else None
        self.min_hits, self.confirm_window = min_hits, confirm_window
        self.min_side_px, self.max_reproj_px = min_side_px, max_reproj_px
        self.max_tilt = math.radians(max_tilt_deg)
        self.min_range, self.max_range = min_range, max_range
        # LiDAR range replaces the marker's when they agree within this
        self.lidar_tol, self.lidar_tol_rel = lidar_tol, lidar_tol_rel
        # a marker's carrier may be a car: faster than a person
        self.max_speed, self.track_gate, self.jump_frames = max_speed, track_gate, jump_frames
        # unseen this long -> SEARCHING (the controller tolerates 0.6 s gaps)
        self.lost_after, self.reid_window = lost_after, reid_window
        self.resume_hits, self.forget_after = resume_hits, forget_after
        self.tracks = {}                       # marker_id -> MarkerTrack
        self.selected_id = -1
        self.needs_resume = False              # selected marker was lost; waiting for resume_hits
        self.resume_seen = []
        self.lost_at = None                    # when the selected marker was last seen before a loss
        self.refound = None                    # (marker_id, seconds unseen) when a lost marker resumed
        self.stats = {"rejected": 0, "gated": 0, "duplicate": 0}
        self.last_reject = None

    # ---- geometry ----
    def observe(self, marker_id, corners, K, dist, size, cam_t, cam_R, pts=None):
        """Locate one detection in base_footprint. cam_t (3,), cam_R (3, 3): the
        camera optical frame in base_footprint. pts: base-frame scan points or None.
        -> MarkerObs; obs.reject says why it must not count (None = fine)."""
        c = np.asarray(corners, dtype=np.float64).reshape(4, 2)
        side = float(np.mean(np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1)))
        o = MarkerObs(marker_id, c.ravel(), side_px=side)
        if self.allowed is not None and marker_id not in self.allowed:
            o.reject = f"id {marker_id} not in the allowed ids"
            return o
        if side < self.min_side_px:
            o.reject = f"too small ({side:.0f} px)"
            return o
        fx, fy, cx, cy = K
        u, v = c.mean(axis=0)
        xy_m = None
        if size > 0:
            pose = marker_pose(c, size, K, dist)
            if pose is None:
                o.reject = "no pose"
                return o
            t, R, err, sure = pose
            if err > self.max_reproj_px:
                o.reject = f"poor fit ({err:.1f} px)"
                return o
            n = R[:, 2]
            tilt = math.acos(max(-1.0, min(1.0, float(-n @ t) / float(np.linalg.norm(t)))))
            if tilt > self.max_tilt:
                o.reject = f"seen too edge-on ({math.degrees(tilt):.0f} deg)"
                return o
            p = cam_R @ t + cam_t
            xy_m = (float(p[0]), float(p[1]))
            if sure:                                     # else NaN: see marker_pose
                nb = cam_R @ n
                o.heading = math.atan2(-nb[1], -nb[0])  # the carrier faces away from the marker's normal
            u, v = fx * t[0] / t[2] + cx, fy * t[1] / t[2] + cy      # the true centre
        d = cam_R @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        b_cam = math.atan2(d[1], d[0])                   # bearing from the CAMERA
        xy = xy_m
        o.source = "marker" if xy_m is not None else ""
        if pts is not None and len(pts[0]):
            half = max(math.radians(1.0), math.atan(side / 2.0 / fx) * 1.5)
            xl = range_at(pts[0], pts[1], (cam_t[0], cam_t[1]), b_cam, half, 0.5, 2,
                          reject_extended=False)
            if xl is not None:
                r_l = math.hypot(xl[0] - cam_t[0], xl[1] - cam_t[1])
                along = (cam_t[0] + r_l * math.cos(b_cam), cam_t[1] + r_l * math.sin(b_cam))
                if xy_m is None:
                    xy, o.source = along, "lidar"
                else:
                    r_m = math.hypot(xy_m[0] - cam_t[0], xy_m[1] - cam_t[1])
                    if abs(r_l - r_m) <= self.lidar_tol + self.lidar_tol_rel * r_m:
                        xy, o.source = along, "lidar"
        if xy is not None:
            r = math.hypot(*xy)
            if not (self.min_range <= r <= self.max_range):
                o.reject = f"out of range ({r:.1f} m)"
                return o
            o.xy, o.bearing = xy, math.atan2(xy[1], xy[0])
        else:
            o.bearing = b_cam                            # no range: the camera ray's bearing
        return o

    # ---- selection ----
    def select(self, tid):
        """-> (ok, reason)."""
        t = next((t for t in self.tracks.values() if t.id == tid), None)
        if t is None:
            return False, "no such marker"
        if not t.confirmed:
            return False, "not confirmed yet (seen too few times)"
        if not t.ever_ranged:
            return False, "no range yet (marker size unset and no LiDAR return)"
        self.selected_id, self.needs_resume, self.resume_seen, self.refound = tid, False, [], None
        self.lost_at = None
        return True, None

    def clear(self):
        self.selected_id, self.needs_resume, self.resume_seen, self.refound = -1, False, [], None
        self.lost_at = None

    def selected(self):
        return next((t for t in self.tracks.values() if t.id == self.selected_id), None)

    def state(self, now):
        if self.selected_id < 0:
            return NONE
        t = self.selected()
        if t is None or t.t_seen is None:
            return LOST
        unseen = now - t.t_seen
        if self.needs_resume or unseen > self.lost_after:
            lost = self.lost_at if self.lost_at is not None else t.t_seen
            return SEARCHING if t.visible or now - lost <= self.reid_window else LOST
        return TRACKING

    def search_hint(self, now):
        t = self.selected()
        if t is None or t.visible:
            return None
        return t.predicted_pos(now)

    # ---- per frame ----
    def update(self, obs, now, T=None):
        """One processed camera frame. obs: [MarkerObs] from observe(); T: odom<-base
        (tx, ty, yaw) or None. -> (tracks, target): target = (x, y, source) in
        base_footprint for the selected marker, or None."""
        self.refound = None
        sel = self.selected()
        # a loss is judged BEFORE this frame's sightings: after a detector stall the
        # first frame back must not skip the resume check
        if sel is not None and not self.needs_resume and sel.t_seen is not None \
                and now - sel.t_seen > self.lost_after:
            self.needs_resume, self.resume_seen, self.lost_at = True, [], sel.t_seen
        for t in self.tracks.values():
            t.visible, t.xy = False, None
        seen = set()
        for o in obs:
            if o.reject:
                self.stats["rejected"] += 1
                self.last_reject = f"marker {o.marker_id}: {o.reject}"
                continue
            t = self.tracks.get(o.marker_id)
            if t is None:
                t = self.tracks[o.marker_id] = MarkerTrack(o.marker_id, self.id_base + o.marker_id, now)
            if o.marker_id in seen:                # the same ID twice in one frame: keep the first
                self.stats["duplicate"] += 1
                continue
            op = to_odom(T, *o.xy) if (o.xy is not None and T is not None) else None
            active = t.t_seen is not None and now - t.t_seen <= self.lost_after
            pred = t.predicted_pos(now)
            if op is not None and pred is not None and active:
                gate = self.track_gate + self.max_speed * (now - t.t_pos)
                if math.hypot(op[0] - pred[0], op[1] - pred[1]) > gate:
                    t.jumps += 1
                    if t.jumps < self.jump_frames:
                        self.stats["gated"] += 1
                        continue
            t.jumps = 0
            seen.add(o.marker_id)
            t.visible, t.t_seen, t.corners = True, now, o.corners
            t.sightings = [s for s in t.sightings if now - s <= self.confirm_window] + [now]
            if len(t.sightings) >= self.min_hits:
                t.confirmed = True
            t.bearing, t.heading, t.source = o.bearing, o.heading, o.source
            if o.xy is not None:
                t.xy, t.range, t.ever_ranged = o.xy, math.hypot(*o.xy), True
                if op is not None:
                    self._fix(t, op, now, reset=not active)
            else:
                t.range = NAN
        # forget: unconfirmed after the confirmation window, others after forget_after
        for mid in [m for m, t in self.tracks.items()
                    if t.id != self.selected_id and not t.visible and t.t_seen is not None
                    and now - t.t_seen > (self.forget_after if t.confirmed else self.confirm_window)]:
            del self.tracks[mid]

        target = None
        t = self.selected()
        if t is not None:
            if self.needs_resume and t.visible:
                self.resume_seen = [s for s in self.resume_seen if now - s <= 0.5] + [now]
                if len(self.resume_seen) >= self.resume_hits:
                    self.needs_resume = False
                    self.refound = (t.marker_id, now - self.lost_at)
                    self.lost_at = None
            if not self.needs_resume and t.visible and t.xy is not None:
                target = (t.xy[0], t.xy[1], "marker+lidar" if t.source == "lidar" else "marker")
        return list(self.tracks.values()), target

    def _fix(self, t, o, now, reset=False):
        if t.pos is not None and not reset and now - t.t_pos > 0.05:
            dt = now - t.t_pos
            vx, vy = (o[0] - t.pos[0]) / dt, (o[1] - t.pos[1]) / dt
            sp = math.hypot(vx, vy)
            if sp > self.max_speed:
                vx, vy = vx * self.max_speed / sp, vy * self.max_speed / sp
            a = 0.3
            t.vel = (a * vx + (1 - a) * t.vel[0], a * vy + (1 - a) * t.vel[1])
        elif reset:
            t.vel = (0.0, 0.0)
        t.pos, t.t_pos = o, now
