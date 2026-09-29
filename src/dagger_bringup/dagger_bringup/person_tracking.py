"""Multi-person tracking logic for click-to-select following -- no ROS in here.

Kept free of ROS so it can be unit-tested frame by frame (tools/test_person_tracking.py)
and replayed from bags. The node wrapper is person_tracker.py.

Phase 1 (this file today): image-space association by box overlap (IoU) with a
Hungarian assignment, so every visible person has a stable ID the operator can
click. Later phases add a LiDAR/odom position and an appearance (re-ID) term to
the same cost matrix; the Track/Tracker interface stays.

Rules that matter for safety (they hold in every phase):
  * IDs are never reused, so "follow #7" can never silently become someone else.
  * The tracker never moves the selection to another track by itself. If the
    selected track is dropped, the selection is kept as an ID with state LOST and
    only an explicit select() or clear() changes it.
"""
import numpy as np
from scipy.optimize import linear_sum_assignment

NONE, TRACKING, COASTING, LOST = "NONE", "TRACKING", "COASTING", "LOST"


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


class Track:
    __slots__ = ("id", "box", "vel", "score", "visible", "age", "missed")

    def __init__(self, tid, box, score):
        self.id = tid
        self.box = np.asarray(box, float)
        self.vel = np.zeros(4)          # box change per frame (constant-velocity prediction)
        self.score = float(score)
        self.visible = True
        self.age = 1
        self.missed = 0

    def predicted(self):
        """Box expected in the next frame (velocity damped while coasting)."""
        return self.box + self.vel * (0.5 ** self.missed)


class Tracker:

    def __init__(self, iou_min=0.2, new_track_score=0.5, max_missed=10, vel_gain=0.5):
        self.iou_min = iou_min                  # below this a box is not the same person
        self.new_track_score = new_track_score  # weaker detections only extend tracks
        self.max_missed = max_missed            # frames a track survives unseen
        self.vel_gain = vel_gain                # smoothing of the box velocity
        self.tracks = []
        self.next_id = 1
        self.selected_id = -1
        self.selected_lost = False

    # ---- operator ----
    def select(self, tid):
        """Select an EXISTING track. Returns False (and changes nothing) otherwise."""
        if any(t.id == tid for t in self.tracks):
            self.selected_id, self.selected_lost = int(tid), False
            return True
        return False

    def clear(self):
        self.selected_id, self.selected_lost = -1, False

    def selected(self):
        for t in self.tracks:
            if t.id == self.selected_id:
                return t
        return None

    def state(self):
        if self.selected_id < 0:
            return NONE
        if self.selected_lost:
            return LOST
        t = self.selected()
        return TRACKING if t is not None and t.visible else COASTING

    # ---- per frame ----
    def update(self, boxes, scores):
        """One camera frame of detections -> updated tracks (list of Track)."""
        boxes = np.asarray(boxes, float).reshape(-1, 4)
        scores = np.asarray(scores, float).reshape(-1)
        pred = np.array([t.predicted() for t in self.tracks]).reshape(-1, 4)
        iou = iou_matrix(pred, boxes)
        matched_t, matched_d = set(), set()
        if iou.size:
            rows, cols = linear_sum_assignment(-iou)
            for r, c in zip(rows, cols):
                if iou[r, c] >= self.iou_min:
                    t = self.tracks[r]
                    new = boxes[c]
                    t.vel = self.vel_gain * (new - t.box) + (1 - self.vel_gain) * t.vel
                    t.box, t.score = new, float(scores[c])
                    t.visible, t.missed = True, 0
                    t.age += 1
                    matched_t.add(r)
                    matched_d.add(c)
        for i, t in enumerate(self.tracks):
            if i not in matched_t:
                t.box = t.predicted()
                t.visible, t.score = False, 0.0
                t.missed += 1
                t.age += 1
        for c in range(len(boxes)):
            if c not in matched_d and scores[c] >= self.new_track_score:
                self.tracks.append(Track(self.next_id, boxes[c], scores[c]))
                self.next_id += 1
        alive = []
        for t in self.tracks:
            if t.missed <= self.max_missed:
                alive.append(t)
            elif t.id == self.selected_id:
                self.selected_lost = True     # keep the ID; never hand it to someone else
        self.tracks = alive
        return self.tracks
