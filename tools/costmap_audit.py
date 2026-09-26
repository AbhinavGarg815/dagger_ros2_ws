#!/usr/bin/env python3
"""Are the costmap's obstacles real right now? NON-MOTION: reads only.

    python3 tools/costmap_audit.py [local|global]

Classifies every LETHAL cell (value 100) of the costmap by whether a CURRENT
return from /scan (ground-segmented) or /scan_band (old band) lies within 0.2 m:
  live   -- something is there now
  stale  -- nothing is there now: a leftover mark
Inflation (99 = inscribed halo, 1-98 = cost gradient) is reported separately:
it surrounds real obstacles and is not stale, but it is most of what you SEE.
Run it at the moment stale-looking obstacles appear, e.g. after moving the rover.

Lesson from writing it: an earlier draft counted >= 99 as obstacles, so the
inscribed halo (up to 0.58 m from a return) was misread as 1,100 "stale" cells.
"""
import math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
                       qos_profile_sensor_data)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros

which = sys.argv[1] if len(sys.argv) > 1 else "local"
rclpy.init(); n = Node("costmap_audit")
tl = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST, depth=1)
st = {}
n.create_subscription(OccupancyGrid, f"/{which}_costmap/costmap", lambda m: st.__setitem__("cm", m), tl)
for k in ("/scan", "/scan_band"):
    n.create_subscription(LaserScan, k, lambda m, k=k: st.__setitem__(k, m), qos_profile_sensor_data)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
e = time.time() + 4
while time.time() < e:
    rclpy.spin_once(n, timeout_sec=0.05)
if "cm" not in st or "/scan" not in st:
    print("no costmap or scan -- is T2 up?"); sys.exit(1)
cm = st["cm"]
tf = buf.lookup_transform(cm.header.frame_id, "base_footprint", rclpy.time.Time())
rx, ry = tf.transform.translation.x, tf.transform.translation.y
q = tf.transform.rotation
yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
g = np.asarray(cm.data, dtype=np.int16).reshape(cm.info.height, cm.info.width)
res = cm.info.resolution; ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
iy, ix = np.nonzero(g == 100)
wx, wy = ox + (ix + 0.5) * res, oy + (iy + 0.5) * res
dx, dy = wx - rx, wy - ry
bx = math.cos(yaw) * dx + math.sin(yaw) * dy
by = -math.sin(yaw) * dx + math.cos(yaw) * dy


def pts(k):
    s = st.get(k)
    if s is None:
        return np.zeros((0, 2))
    r = np.asarray(s.ranges); a = s.angle_min + np.arange(len(r)) * s.angle_increment
    ok = np.isfinite(r)
    return np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]


def near(P):
    out = np.zeros(len(bx), bool)
    if not len(P):
        return out
    for i in range(0, len(bx), 200):
        d = np.hypot(bx[i:i + 200, None] - P[None, :, 0], by[i:i + 200, None] - P[None, :, 1])
        out[i:i + 200] = d.min(1) < 0.2
    return out


ls, lb = near(pts("/scan")), near(pts("/scan_band"))
stale = ~(ls | lb)
print(f"{which} costmap ({cm.header.frame_id}, {res:.2f} m cells)")
print(f"  lethal cells        {len(bx):6d}   live {int((ls | lb).sum())}  "
      f"(of which only the low /scan layer: {int((ls & ~lb).sum())})   STALE {int(stale.sum())}")
print(f"  inscribed halo (99) {int((g == 99).sum()):6d}   cost gradient (1-98) {int(((g > 0) & (g < 99)).sum())}")
if stale.any():
    sr = np.hypot(bx[stale], by[stale]); sb = np.degrees(np.arctan2(by[stale], bx[stale]))
    print(f"  stale cells: {sr.min():.1f}-{sr.max():.1f} m away; ahead {int((abs(sb) < 60).sum())}, "
          f"sides {int(((abs(sb) >= 60) & (abs(sb) < 120)).sum())}, behind {int((abs(sb) >= 120).sum())}")
