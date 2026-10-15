"""Following for the operator app: who the camera sees, and who is picked. NO ROS.

The trackers (dagger_bringup person_tracker / marker_tracker) publish
PersonTracks and MarkerTracks per camera frame -- the same the follow web page
(ui_server.py) draws. Here they become the contract's Targets (for the video
channel) and FollowStatus (in RoverState). Messages are read by attribute, so
the tests use plain objects. Tested offline by test/test_follow.py.
"""
import math

from . import api_pb2 as pb

TARGET_STATES = {
    "TRACKING": pb.TARGET_STATE_TRACKING,
    "HOLD_LIDAR": pb.TARGET_STATE_HOLD,
    "LOST": pb.TARGET_STATE_LOST,
}


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _range(v):
    return float(v) if v is not None and math.isfinite(v) and v > 0 else 0.0


def geometry(tracks, markers, default=(640, 480, 0)):
    """(width, height, rotation_deg) of the camera picture, from whichever tracker spoke."""
    for m in (tracks, markers):
        if m is not None and m.image_width and m.image_height:
            return m.image_width, m.image_height, m.image_rotation_deg
    return default


def targets_msg(tracks, markers):
    """pb.Targets from the latest PersonTracks and MarkerTracks (either may be None)."""
    w, h, rot = geometry(tracks, markers)
    head = tracks if tracks is not None else markers
    out = pb.Targets(stamp_s=stamp_s(head.header) if head is not None else 0.0,
                     width=w, height=h, rotation_deg=rot, selected_id=selected_id(tracks, markers))
    if tracks is not None:
        for t in tracks.tracks:
            out.targets.add(id=t.id, kind=pb.TARGET_KIND_PERSON, outline=[t.x1, t.y1, t.x2, t.y2],
                            selectable=bool(t.ranged), range_m=_range(t.range_m), visible=bool(t.visible))
    if markers is not None:
        for t in markers.tracks:
            out.targets.add(id=t.id, kind=pb.TARGET_KIND_MARKER, outline=list(t.corners),
                            selectable=bool(t.confirmed), range_m=_range(t.range_m), visible=bool(t.visible))
    return out


def selected_id(tracks, markers):
    """The picked target: a marker when one is picked, else the person tracker's pick, else -1."""
    if markers is not None and markers.selected_id >= 0:
        return markers.selected_id
    if tracks is not None and tracks.selected_id >= 0:
        return tracks.selected_id
    return -1


def follow_status(tracks, markers):
    """pb.FollowStatus for RoverState, or None when nothing is picked."""
    sid = selected_id(tracks, markers)
    if sid < 0:
        return None
    on_marker = markers is not None and markers.selected_id == sid
    src = markers if on_marker else tracks
    picked = next((t for t in src.tracks if t.id == sid), None)
    return pb.FollowStatus(target_id=sid,
                           state=TARGET_STATES.get(src.target_state, pb.TARGET_STATE_UNSPECIFIED),
                           range_m=_range(picked.range_m) if picked is not None else 0.0)


def is_selectable(tracks, markers, target_id):
    """True when `target_id` is in the latest tracks and may be picked (a ranged
    person, a confirmed marker) -- what the trackers themselves would accept."""
    t = targets_msg(tracks, markers)
    return any(x.id == target_id and x.selectable for x in t.targets)
