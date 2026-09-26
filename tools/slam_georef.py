#!/usr/bin/env python3
"""Tie a SLAM map to the Earth: fit slam_map -> local ENU from a mapping drive.

  python3 tools/slam_georef.py <bag> <poses.csv> <out_geo.yaml>
  python3 tools/slam_georef.py ~/dagger_bags/slam_lane3_164840 \
      ~/dagger_bags/replay/lane3fix/poses.csv src/dagger_bringup/maps/lane3fix_geo.yaml

The ArduPilot local frame cannot be used as the anchor: its origin is re-set on
every FCU boot. So the anchor is a lat/lon DATUM (mean of the drive) with ENU
axes, and the result is a 2D rigid transform  p_enu = R(yaw) * p_slam + t.

Rotation comes from the dual-antenna heading (/gps/heading_imu, ~0.5 deg) rather
than from the position fit, which on a ~20 m lane with ~1 m GNSS error is only
good to a few degrees. Translation is then the least-squares fit of SLAM poses
to GNSS positions (/mavros/global_position/global). The residual printed is the
GNSS-vs-SLAM disagreement -- it bounds how well a GPS-derived start pose (and a
lat/lon goal) can land on this map.
"""
import bisect
import csv
import math
import os
import sys

import numpy as np

A_WGS84 = 6378137.0
E2_WGS84 = 6.69437999014e-3


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def enu_scale(lat0_deg):
    """Metres per degree (east, north) at lat0 -- fine for maps under a few km."""
    lat = math.radians(lat0_deg)
    s = 1 - E2_WGS84 * math.sin(lat) ** 2
    n = A_WGS84 / math.sqrt(s)                    # prime vertical radius
    m = A_WGS84 * (1 - E2_WGS84) / s ** 1.5       # meridian radius
    return math.radians(1) * n * math.cos(lat), math.radians(1) * m


def read_bag(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, NavSatFix

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(
        topics=["/mavros/global_position/global", "/gps/heading_imu"]))
    fixes, heads = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == "/gps/heading_imu":
            m = deserialize_message(raw, Imu)
            heads.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                          yaw_of(m.orientation)))
        else:
            m = deserialize_message(raw, NavSatFix)
            if m.status.status >= 0:
                fixes.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                              m.latitude, m.longitude))
    return fixes, heads


def nearest(series, t, tol):
    ts = [s[0] for s in series]
    i = bisect.bisect_left(ts, t)
    best = min((j for j in (i - 1, i) if 0 <= j < len(series)),
               key=lambda j: abs(ts[j] - t), default=None)
    if best is None or abs(ts[best] - t) > tol:
        return None
    return series[best]


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    bag, poses_csv, out = (os.path.expanduser(a) for a in sys.argv[1:])
    fixes, heads = read_bag(bag)
    with open(poses_csv) as f:
        poses = [tuple(float(v) for v in row) for row in list(csv.reader(f))[1:]]
    if not fixes or not heads or len(poses) < 10:
        print(f"not enough data: {len(fixes)} fixes, {len(heads)} headings, {len(poses)} poses")
        return 1

    lat0 = sum(f[1] for f in fixes) / len(fixes)
    lon0 = sum(f[2] for f in fixes) / len(fixes)
    kx, ky = enu_scale(lat0)

    dyaw = []
    P, Q = [], []
    for t, x, y, yaw in poses:
        h = nearest(heads, t, 0.2)
        if h is not None:
            dyaw.append(math.remainder(h[1] - yaw, 2 * math.pi))
        g = nearest(fixes, t, 0.2)
        if g is not None:
            P.append((x, y))
            Q.append(((g[2] - lon0) * kx, (g[1] - lat0) * ky))
    # circular mean of the heading offsets
    rot = math.atan2(sum(math.sin(d) for d in dyaw), sum(math.cos(d) for d in dyaw))
    spread = max(abs(math.degrees(math.remainder(d - rot, 2 * math.pi))) for d in dyaw)
    P, Q = np.array(P), np.array(Q)
    R = np.array([[math.cos(rot), -math.sin(rot)], [math.sin(rot), math.cos(rot)]])
    t = (Q - P @ R.T).mean(axis=0)
    res = np.linalg.norm(P @ R.T + t - Q, axis=1)

    # for comparison: rotation from positions alone
    pc, qc = P - P.mean(0), Q - Q.mean(0)
    pos_rot = math.degrees(math.atan2((pc[:, 0] * qc[:, 1] - pc[:, 1] * qc[:, 0]).sum(),
                                      (pc * qc).sum()))

    print(f"datum {lat0:.7f}, {lon0:.7f} | {len(dyaw)} heading pairs, {len(P)} position pairs")
    print(f"rotation slam_map->ENU {math.degrees(rot):+.2f} deg from heading (max dev {spread:.1f} deg); "
          f"position-only fit would say {pos_rot:+.2f} deg")
    print(f"translation ({t[0]:+.2f}, {t[1]:+.2f}) m | GNSS-vs-SLAM residual rms {np.sqrt((res ** 2).mean()):.2f} m, "
          f"max {res.max():.2f} m")
    with open(out, "w") as f:
        f.write(f"# slam_map -> local ENU at the datum:  p_enu = R(yaw) * p_map + t\n"
                f"# made by tools/slam_georef.py from {os.path.basename(bag.rstrip('/'))}\n"
                f"datum_lat: {lat0:.9f}\ndatum_lon: {lon0:.9f}\n"
                f"yaw_deg: {math.degrees(rot):.4f}\ntx: {t[0]:.4f}\nty: {t[1]:.4f}\n"
                f"residual_rms_m: {np.sqrt((res ** 2).mean()):.3f}\nresidual_max_m: {res.max():.3f}\n")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
