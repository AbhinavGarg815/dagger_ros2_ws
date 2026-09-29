"""Multi-person tracking for click-to-select following -- no ROS in here.

Kept free of ROS so it can be unit-tested frame by frame
(tools/test_person_tracking.py) and replayed from bags. The node wrapper is
person_tracker.py, which supplies each frame's detections already located:
camera bearing + LiDAR range -> an (x, y) in base_footprint, as target_selector
has done since 2026-09-25 (range_at / lidar cluster rules below are its logic,
moved here unchanged so both paths share one implementation).

EVERY person is tracked, with
  * the box in the image (for the operator's click and the display),
  * a position + velocity in the ODOM frame when the LiDAR ranges them.
Association (Hungarian): where both a track and a detection have an odom
position, the POSITION decides -- a detection outside the track's gate cannot
match, however well the boxes overlap. That is what keeps two people apart when
they cross in the image at different distances (image overlap alone swaps them).
Without a position on either side, box overlap (IoU) decides.

The SELECTED person is additionally held on the LiDAR while the camera cannot
see them (beside / behind the rover), for at most lidar_only_timeout, on the
cluster nearest the predicted position -- refusing clusters too wide to be a
person (walls, vehicles).

Safety rules (unit-tested):
  * IDs are never reused, so "follow #7" can never silently become someone else.
  * The tracker never moves the selection to another track by itself. A dropped
    selection becomes LOST; only select() / clear() change it.
  * No range, no target: the selected person is reported as a target only with
    a LiDAR-backed position within [min_range, max_range].
"""
import math

import numpy as np
from scipy.optimize import linear_sum_assignment

NONE, TRACKING, HOLD_LIDAR, COASTING, LOST = "NONE", "TRACKING", "HOLD_LIDAR", "COASTING", "LOST"
INF = 1e9


def iou_matrix(a, b):
    """IoU between every box in a (N,4) and b (M,4), boxes as x1,y1,x2,y2."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    a, b = np.asarray(a, float), np.asarray(b, float)
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    union = area_a[:, None] + area_b[None, :] - inter
    return np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)


# ---------------- geometry (base_footprint unless stated) ----------------

def to_odom(T, x, y):
    tx, ty, yaw = T
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y


def to_base(T, x, y):
    tx, ty, yaw = T
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - tx, y - ty
    return c * dx + s * dy, -s * dx + c * dy


def scan_xy(ranges, angle_min, angle_increment, range_min, range_max):
    """LaserScan (in base_footprint) -> valid points px, py."""
    r = np.asarray(ranges, dtype=np.float32)
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    idx = np.nonzero(ok)[0]
    ang = angle_min + idx * angle_increment
    return r[idx] * np.cos(ang), r[idx] * np.sin(ang)


def range_at(px, py, origin, bearing, half_width, cluster_depth=0.6, min_cluster=2):
    """Nearest LiDAR cluster inside +-half_width of `bearing` seen from the camera
    `origin` -> its (x, y) in base_footprint, or None. (target_selector.range_at)"""
    if len(px) == 0:
        return None
    rel = np.arctan2(py - origin[1], px - origin[0])
    d = np.abs(np.arctan2(np.sin(rel - bearing), np.cos(rel - bearing)))
    sel = d <= half_width
    if not sel.any():
        return None
    sx, sy = px[sel], py[sel]
    rng = np.hypot(sx, sy)                       # distance from the ROVER
    body = rng <= rng.min() + cluster_depth      # one person, not the wall behind
    if body.sum() < min_cluster:
        return None
    return float(np.median(sx[body])), float(np.median(sy[body]))


def lidar_cluster(ox, oy, pred, gate, min_points=3, max_width=0.9):
    """Person-sized cluster nearest `pred` (all in ODOM) -> ((x, y), None) or
    (None, reason). None is the safe answer: anything ambiguous ends the hold
    rather than guessing. (target_selector.lidar_fix)"""
    if len(ox) == 0:
        return None, "no scan points"
    d = np.hypot(ox - pred[0], oy - pred[1])
    sel = d <= gate
    if sel.sum() < min_points:
        return None, "nothing near the prediction"
    k = np.argmin(np.where(sel, d, np.inf))
    body = sel & (np.hypot(ox - ox[k], oy - oy[k]) <= max_width / 2 + 0.1)
    if body.sum() < min_points:
        return None, "too few points"
    bx, by = ox[body], oy[body]
    width = math.hypot(bx.max() - bx.min(), by.max() - by.min())
    near_all = sel & (np.hypot(ox - ox[k], oy - oy[k]) <= 1.5)
    span = math.hypot(ox[near_all].max() - ox[near_all].min(),
                      oy[near_all].max() - oy[near_all].min())
    if width > max_width or span > 2.0 * max_width:
        return None, f"cluster {max(width, span):.2f} m wide -- not a person"
    return (float(np.median(bx)), float(np.median(by))), None


# ---------------- tracks ----------------

class Detection:
    """One person in one frame. xy: LiDAR-backed (x, y) in base_footprint or None."""
    __slots__ = ("box", "score", "bearing", "xy")

    def __init__(self, box, score, bearing=float("nan"), xy=None):
        self.box = np.asarray(box, float)
        self.score = float(score)
        self.bearing = float(bearing)
        self.xy = xy


class Track:
    __slots__ = ("id", "box", "vbox", "score", "visible", "age", "missed", "hits", "disputes",
                 "pos", "vel", "t_pos", "t_cam", "held", "bearing", "range")

    def __init__(self, det, now):
        self.id = None                  # public id, given on confirmation (min_hits)
        self.hits = 1                   # camera matches so far
        self.disputes = 0               # consecutive matches whose LiDAR range disagreed
        self.box = det.box.copy()
        self.vbox = np.zeros(4)         # box change per frame
        self.score = det.score
        self.visible = True
        self.age = 1
        self.missed = 0
        self.pos = None                 # odom (x, y) of the last position fix
        self.vel = (0.0, 0.0)           # odom m/s
        self.t_pos = now
        self.t_cam = now                # last camera confirmation
        self.held = False               # LiDAR-held THIS frame (selected only)
        self.bearing = det.bearing
        self.range = float("nan")

    def predicted_box(self):
        return self.box + self.vbox * (0.5 ** self.missed)

    def predicted_pos(self, now):
        if self.pos is None:
            return None
        dt = now - self.t_pos
        return self.pos[0] + self.vel[0] * dt, self.pos[1] + self.vel[1] * dt


class Tracker:

    def __init__(self, iou_min=0.2, new_track_score=0.5, max_missed=20, vel_gain=0.5,
                 track_gate=0.6, max_person_speed=1.8, lidar_only_timeout=5.0,
                 min_track_points=3, max_cluster_width=0.9, min_range=1.0, max_range=15.0,
                 other_person_radius=0.6, high_score=0.45, iou_strong=0.4, dispute_frames=3,
                 dup_iou=0.5, min_hits=3, tentative_max_missed=2):
        self.iou_min = iou_min                  # without positions, below this is not the same person
        self.new_track_score = new_track_score  # weaker detections only extend tracks
        self.max_missed = max_missed            # camera frames a track survives unseen
        self.vel_gain = vel_gain                # box velocity smoothing
        self.track_gate = track_gate            # m around the predicted position ...
        self.max_person_speed = max_person_speed  # ... plus what a person covers since
        self.lidar_only_timeout = lidar_only_timeout
        self.min_track_points = min_track_points
        self.max_cluster_width = max_cluster_width
        self.min_range, self.max_range = min_range, max_range
        # a hold cluster within this of another person's fresh position is theirs
        self.other_person_radius = other_person_radius
        # --- ID stability (live 2026-09-29: 25 ids in 100 s for 1-3 people) ---
        # Two-stage matching (ByteTrack): detections >= high_score first; weaker
        # ones (side-on / back views score lower) may then only EXTEND tracks.
        self.high_score = high_score
        # A match the LiDAR range contradicts (range_at flipped to legs, background,
        # a pole) is still the same person if the boxes overlap this much: keep the
        # id, ignore that range; re-anchor to it only after dispute_frames in a row.
        self.iou_strong, self.dispute_frames = iou_strong, dispute_frames
        # An unmatched detection overlapping a track this much is a duplicate,
        # not a new person.
        self.dup_iou = dup_iou
        # two detections whose LiDAR positions are closer than this got ONE cluster
        self.same_cluster = 0.35
        # New tracks are TENTATIVE (no id, not shown, not clickable) until matched
        # min_hits times; a tentative one dies after tentative_max_missed misses.
        self.min_hits, self.tentative_max_missed = min_hits, tentative_max_missed
        self.tracks = []
        self.next_id = 1
        self.selected_id = -1
        self.selected_lost = False
        self.hold_refusal = None                # last reason the LiDAR hold refused, for logs

    # ---- operator ----
    def select(self, tid):
        """Select an EXISTING track. Returns False (and changes nothing) otherwise."""
        if any(t.id == tid for t in self.tracks if t.id is not None):
            self.selected_id, self.selected_lost = int(tid), False
            return True
        return False

    def clear(self):
        self.selected_id, self.selected_lost = -1, False

    def selected(self):
        for t in self.tracks:
            if t.id is not None and t.id == self.selected_id:
                return t
        return None

    def confirmed(self):
        """Tracks with a public id (what the UI shows and the operator can click)."""
        return [t for t in self.tracks if t.id is not None]

    def state(self):
        if self.selected_id < 0:
            return NONE
        if self.selected_lost:
            return LOST
        t = self.selected()
        if t is None:
            return LOST
        if t.visible:
            return TRACKING
        return HOLD_LIDAR if t.held else COASTING

    def gate(self, dt):
        return self.track_gate + min(self.max_person_speed * dt, 1.0)

    # ---- per frame ----
    def update(self, dets, now, T=None, scan=None):
        """One camera frame.

        dets: [Detection]; now: seconds (monotonic); T: odom<-base (tx, ty, yaw) or
        None (no odometry -> image-only association); scan: (px, py) base-frame
        LiDAR points or None (no LiDAR hold).
        Returns (confirmed tracks, target) where target is (x, y, source) in
        base_footprint for the selected person, or None.
        """
        # One LiDAR cluster cannot be two people. range_at takes the NEAREST
        # cluster in a box's direction, so a person partly behind another gets the
        # front person's range. Where two detections got (nearly) the same position,
        # the larger box -- the nearer person -- keeps it; the other is unranged.
        # (scene test 2026-09-29: A walking behind B was re-anchored onto B.)
        for a_ in range(len(dets)):
            for b_ in range(a_ + 1, len(dets)):
                da, db = dets[a_], dets[b_]
                if da.xy is not None and db.xy is not None and \
                        math.hypot(da.xy[0] - db.xy[0], da.xy[1] - db.xy[1]) < self.same_cluster:
                    ha, hb = da.box[3] - da.box[1], db.box[3] - db.box[1]
                    (db if ha >= hb else da).xy = None
        det_o = [to_odom(T, *d.xy) if (T is not None and d.xy is not None) else None
                 for d in dets]
        # stage 1: all tracks x confident detections; stage 2: leftover tracks x weak ones
        high = [j for j, d in enumerate(dets) if d.score >= self.high_score]
        low = [j for j, d in enumerate(dets) if d.score < self.high_score]
        matches = self._associate(list(range(len(self.tracks))), high, dets, det_o, now, T)
        left = [i for i in range(len(self.tracks)) if i not in matches]
        matches.update(self._associate(left, low, dets, det_o, now, T))
        matched_d = {j for j, _ in matches.values()}

        target = None
        unmatched_sel = None
        for i, t in enumerate(self.tracks):
            t.held = False
            if i in matches:
                j, disputed = matches[i]
                d, o = dets[j], det_o[j]
                t.vbox = self.vel_gain * (d.box - t.box) + (1 - self.vel_gain) * t.vbox
                t.box, t.score, t.bearing = d.box.copy(), d.score, d.bearing
                t.visible, t.missed, t.t_cam = True, 0, now
                t.hits += 1
                if t.id is None and t.hits >= self.min_hits:
                    t.id = self.next_id
                    self.next_id += 1
                use_pos = o is not None
                if disputed:
                    # Never re-anchor onto where somebody else is: that range is theirs.
                    theirs = any(x is not t and x.id is not None and x.pos is not None and
                                 math.hypot(o[0] - x.predicted_pos(now)[0],
                                            o[1] - x.predicted_pos(now)[1]) < self.other_person_radius
                                 for x in self.tracks)
                    t.disputes = 0 if theirs else t.disputes + 1
                    if t.disputes >= self.dispute_frames:
                        t.pos, t.vel, t.t_pos = o, (0.0, 0.0), now   # consistently elsewhere: re-anchor
                        t.disputes = 0
                    use_pos = False              # this frame's range is not trusted
                else:
                    t.disputes = 0
                if use_pos:
                    self._fix(t, o, now)
                    t.range = math.hypot(*d.xy)
                elif o is None:
                    t.range = float("nan")
                if (t.id is not None and t.id == self.selected_id and use_pos
                        and self.min_range <= t.range <= self.max_range):
                    target = (d.xy[0], d.xy[1], "camera")
            else:
                t.box = t.predicted_box()
                t.visible, t.score = False, 0.0
                t.missed += 1
                if t.id is not None and t.id == self.selected_id and not self.selected_lost:
                    unmatched_sel = t
            t.age += 1

        # LiDAR hold AFTER every camera match of this frame, so it knows where
        # everybody else is: a cluster another person explains is never taken for
        # the selected one. Without this the growing gate slid the hold onto a
        # bystander 1.5 m away when the target vanished (e.g. walked behind them).
        if unmatched_sel is not None:
            others = [x.pos for x in self.tracks if x is not unmatched_sel
                      and x.pos is not None and x.t_pos == now]
            hold = self._lidar_hold(unmatched_sel, now, T, scan, others)
            if hold is not None:
                target = hold

        boxes_now = [t.box for t in self.tracks]
        for j in high:
            d = dets[j]
            if j in matched_d or d.score < self.new_track_score:
                continue
            if boxes_now and iou_matrix([d.box], boxes_now).max() >= self.dup_iou:
                continue                         # someone already tracked there
            t = Track(d, now)
            if self.min_hits <= 1:
                t.id = self.next_id
                self.next_id += 1
            if det_o[j] is not None:
                t.pos, t.range = det_o[j], math.hypot(*d.xy)
            self.tracks.append(t)
            boxes_now.append(t.box)

        alive = []
        for t in self.tracks:
            limit = self.max_missed if t.id is not None else self.tentative_max_missed
            if t.missed <= limit or t.held:
                alive.append(t)
            elif t.id is not None and t.id == self.selected_id:
                self.selected_lost = True        # keep the ID; never hand it to someone else
        self.tracks = alive
        if self.selected_lost:
            target = None
        return self.confirmed(), target

    def _associate(self, tis, djs, dets, det_o, now, T):
        """Hungarian match of tracks tis to detections djs -> {track: (det, disputed)}."""
        if not tis or not djs:
            return {}
        iou = iou_matrix([self.tracks[i].predicted_box() for i in tis], [dets[j].box for j in djs])
        cost = np.full((len(tis), len(djs)), INF)
        disputed = np.zeros_like(cost, dtype=bool)
        for a, i in enumerate(tis):
            t = self.tracks[i]
            pp = t.predicted_pos(now) if T is not None else None
            g = self.gate(now - t.t_pos)
            for b, j in enumerate(djs):
                if pp is not None and det_o[j] is not None:
                    dist = math.hypot(det_o[j][0] - pp[0], det_o[j][1] - pp[1])
                    if dist <= g:                # position decides; IoU only breaks ties
                        cost[a, b] = 0.7 * dist / g + 0.3 * (1.0 - iou[a, b])
                    elif iou[a, b] >= self.iou_strong:   # range disagrees, boxes agree
                        cost[a, b] = 1.2 + (1.0 - iou[a, b])
                        disputed[a, b] = True
                elif iou[a, b] >= self.iou_min:
                    cost[a, b] = 1.0 + (1.0 - iou[a, b])   # worse than any position match
        out = {}
        rows, cols = linear_sum_assignment(cost)
        for r, c in zip(rows, cols):
            if cost[r, c] < INF:
                out[tis[r]] = (djs[c], bool(disputed[r, c]))
        return out

    def _fix(self, t, o, now):
        """Fold an odom position fix into the track (velocity clamped to walking)."""
        if t.pos is not None:
            dt = max(1e-3, now - t.t_pos)
            vx, vy = (o[0] - t.pos[0]) / dt, (o[1] - t.pos[1]) / dt
            sp = math.hypot(vx, vy)
            if sp > self.max_person_speed:       # a jump, not walking
                vx, vy = vx * self.max_person_speed / sp, vy * self.max_person_speed / sp
            a = 0.3
            t.vel = (a * vx + (1 - a) * t.vel[0], a * vy + (1 - a) * t.vel[1])
        t.pos, t.t_pos = o, now

    def _lidar_hold(self, t, now, T, scan, others=()):
        """Carry the selected person on the LiDAR while the camera cannot see them."""
        if T is None or scan is None or t.pos is None:
            return None
        if now - t.t_cam > self.lidar_only_timeout:
            self.hold_refusal = f"on LiDAR alone for > {self.lidar_only_timeout:.0f} s"
            return None
        px, py = scan
        tx, ty, yaw = T
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy = tx + c * px - s * py, ty + s * px + c * py
        got, why = lidar_cluster(ox, oy, t.predicted_pos(now), self.gate(now - t.t_pos),
                                 self.min_track_points, self.max_cluster_width)
        if got is None:
            self.hold_refusal = why
            return None
        for o in others:
            if math.hypot(got[0] - o[0], got[1] - o[1]) < self.other_person_radius:
                self.hold_refusal = "nearest cluster is another tracked person"
                return None
        bx, by = to_base(T, *got)
        rng = math.hypot(bx, by)
        if not (self.min_range <= rng <= self.max_range):
            self.hold_refusal = f"out of range ({rng:.1f} m)"
            return None
        self._fix(t, got, now)
        t.held, t.range, t.bearing = True, rng, math.atan2(by, bx)
        self.hold_refusal = None
        return bx, by, "lidar"
