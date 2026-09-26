#!/usr/bin/env python3
"""What the costmap scans see ahead of the rover. NON-MOTION: reads only.

    python3 tools/scan_ahead.py [seconds]

Compares /scan (ground-segmented, floor 0.12 m) with /scan_band (old 0.35-0.75
band) in the forward sector +-30 deg, so a low object shows up as a return /scan
has and /scan_band does not. Run once with nothing placed (baseline: flat ground
must NOT appear), then with the object.
"""
import math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
rclpy.init(); n = Node("scan_ahead")
S = {"/scan": [], "/scan_band": []}
for k in S:
    n.create_subscription(LaserScan, k, lambda m, k=k: S[k].append(m), qos_profile_sensor_data)
e = time.time() + DUR
while time.time() < e:
    rclpy.spin_once(n, timeout_sec=0.05)


def ahead(msgs):
    """Per 5-deg bin, the median-over-time nearest return within 10 m."""
    m0 = msgs[0]
    ang = m0.angle_min + np.arange(len(m0.ranges)) * m0.angle_increment
    A = np.stack([np.asarray(m.ranges, dtype=np.float32) for m in msgs])
    A[~np.isfinite(A) | (A > 10)] = np.nan
    med = np.nanmedian(A, axis=0)
    out = []
    for lo in range(-30, 30, 5):
        sel = (ang >= math.radians(lo)) & (ang < math.radians(lo + 5))
        v = med[sel]
        out.append(np.nanmin(v) if np.isfinite(v).any() else np.nan)
    return out


if not S["/scan"] or not S["/scan_band"]:
    print("no scans -- is T2 up?"); sys.exit(1)
a, b = ahead(S["/scan"]), ahead(S["/scan_band"])
print(f"forward sector, nearest return per 5 deg bin (m), {len(S['/scan'])} scans")
print(f"  {'bearing':>9s} {'/scan (seg)':>12s} {'/scan_band':>11s}")
low = 0
for i, lo in enumerate(range(-30, 30, 5)):
    fa = f"{a[i]:.2f}" if np.isfinite(a[i]) else "-"
    fb = f"{b[i]:.2f}" if np.isfinite(b[i]) else "-"
    flag = ""
    if np.isfinite(a[i]) and (not np.isfinite(b[i]) or a[i] < b[i] - 0.3):
        flag = "  <- only the segmented scan sees this (low object, or false ground)"
        low += 1
    print(f"  {lo:+4d}..{lo+5:+3d} {fa:>12s} {fb:>11s}{flag}")
print(f"\n{low} forward bins with a return ONLY in the segmented scan")
