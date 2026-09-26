#!/usr/bin/env python3
"""Does a low obstacle stay in the costmap as the rover closes on it? NON-MOTION.

    python3 tools/low_obstacle_approach.py [seconds]

1. Waits until a LOW object (5-45 cm, 2.5-6 m ahead, |y| < 1 m) is in the
   ground-segmented cloud, locks its position in the odom frame, prints GO.
2. The operator then drives slowly toward it on the RC.
3. Every 0.5 s: distance to the object, LiDAR points still on it, the global and
   local costmap values there -- and how far the fused odometry and the FCU's
   own estimate each say the rover moved, as a check that the zero-velocity
   update is not freezing odometry during genuine motion.
Refuses to guess: if the object is not in view it keeps waiting, it does not
assume a position.
"""
import math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
                       qos_profile_sensor_data)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
import tf2_ros

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90
rclpy.init(); n = Node("low_obstacle_approach")
tl = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST, depth=1)
st = {}
for c in ("local", "global"):
    n.create_subscription(OccupancyGrid, f"/{c}_costmap/costmap", lambda m, c=c: st.__setitem__(c, m), tl)
n.create_subscription(PointCloud2, "/patchworkpp/nonground", lambda m: st.__setitem__("ng", m),
                      qos_profile_sensor_data)
n.create_subscription(Odometry, "/mavros/local_position/odom", lambda m: st.__setitem__("ek3", m),
                      qos_profile_sensor_data)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)


def T(frm, to):
    tf = buf.lookup_transform(to, frm, rclpy.time.Time()); t = tf.transform.translation
    q = tf.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return R, np.array([t.x, t.y, t.z])


def pts(m, to):
    a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
    off = {f.name: f.offset for f in m.fields}
    P = np.c_[[a[:, off[k]:off[k]+4].copy().view(np.float32).ravel() for k in "xyz"]].T
    P = P[np.isfinite(P).all(1)]; R, t = T(m.header.frame_id, to)
    return P @ R.T + t


def spin(d):
    e = time.time() + d
    while time.time() < e:
        rclpy.spin_once(n, timeout_sec=0.02)


print("waiting for a low object ahead ...", flush=True)
box = None
t_wait = time.time()
while box is None and time.time() - t_wait < 120:
    spin(0.5)
    if "ng" not in st:
        continue
    try:
        P = pts(st["ng"], "base_footprint")
    except Exception:
        continue
    S = P[(P[:, 0] > 2.5) & (P[:, 0] < 6) & (np.abs(P[:, 1]) < 1.0) & (P[:, 2] > 0.05) & (P[:, 2] < 0.45)]
    if len(S) >= 10:
        R, t = T("base_footprint", "odom")
        bb = np.median(S, 0); box = (R @ bb + t)[:2]
        print(f"locked: {len(S)} pts, {bb[0]:.2f} m ahead, y {bb[1]:+.2f}, heights "
              f"{S[:, 2].min():.2f}-{S[:, 2].max():.2f} m; odom ({box[0]:.2f}, {box[1]:.2f})")
if box is None:
    print("no low object seen ahead in 120 s -- not guessing"); sys.exit(1)


def cm_at(c, r=0.25):
    cm = st.get(c)
    if not cm:
        return "  -"
    g = np.asarray(cm.data, dtype=np.int16).reshape(cm.info.height, cm.info.width)
    res = cm.info.resolution; ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
    k = int(r / res); i0, j0 = int((box[1] - oy) / res), int((box[0] - ox) / res)
    if not (0 <= i0 < g.shape[0] and 0 <= j0 < g.shape[1]):
        return "out"
    w = g[max(0, i0-k):i0+k+1, max(0, j0-k):j0+k+1]
    return f"{w.max():3d}{'*' if (w == 100).any() else ' '}"


_, t0f = T("base_footprint", "odom")
e0 = st["ek3"].pose.pose.position if "ek3" in st else None
print("\n>>> GO: drive slowly toward it now <<<\n", flush=True)
print("  t    dist  lidar-pts  global  local   moved: fused   FCU     (* = lethal cell within 0.25 m)")
t0 = time.time()
while time.time() - t0 < DUR:
    spin(0.5)
    try:
        _, tf_ = T("base_footprint", "odom")
    except Exception:
        continue
    d = math.hypot(box[0] - tf_[0], box[1] - tf_[1])
    npts = 0
    if "ng" in st:
        Q = pts(st["ng"], "odom")
        npts = int(((np.hypot(Q[:, 0]-box[0], Q[:, 1]-box[1]) < 0.35) & (Q[:, 2] > 0.05)
                    & (Q[:, 2] < 0.6)).sum())
    mf = math.hypot(tf_[0] - t0f[0], tf_[1] - t0f[1])
    me = "   -"
    if e0 is not None and "ek3" in st:
        p = st["ek3"].pose.pose.position; me = f"{math.hypot(p.x - e0.x, p.y - e0.y):5.2f}"
    print(f"  {time.time()-t0:4.1f} {d:5.2f} {npts:6d}     {cm_at('global'):>5s}   {cm_at('local'):>5s}"
          f"         {mf:5.2f}  {me}", flush=True)
