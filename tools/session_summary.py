#!/usr/bin/env python3
"""Summarise a Foxglove/Nav2 goal session from a tools/detour_record.sh bag. NO ROVER.

    python3 tools/session_summary.py <bag>

FCU state changes and messages, each goal (clicked pose, Nav2 status, closest / final distance,
final heading error), in-place turns (commanded / after the collision monitor / actual rate),
the collision monitor's effect on the commands, warnings and errors in /rosout.
"""
import collections, math, sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
TOP = {"/goal_pose": "geometry_msgs/msg/PoseStamped", "/navigate_to_pose/_action/status": "action_msgs/msg/GoalStatusArray",
       "/odometry/filtered": "nav_msgs/msg/Odometry", "/cmd_vel_nav_raw": "geometry_msgs/msg/Twist",
       "/cmd_vel_nav": "geometry_msgs/msg/Twist", "/rosout": "rcl_interfaces/msg/Log", "/mavros/state": "mavros_msgs/msg/State"}
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
t0 = rd.get_metadata().starting_time.nanoseconds * 1e-9
rd.set_filter(rosbag2_py.StorageFilter(topics=list(TOP)))
T = {k: get_message(v) for k, v in TOP.items()}
D = collections.defaultdict(list)
while rd.has_next():
    tp, d, ts = rd.read_next()
    D[tp].append((ts * 1e-9 - t0, deserialize_message(d, T[tp])))
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
fo = np.array([(t, m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation),
                m.twist.twist.linear.x, m.twist.twist.angular.z) for t, m in D["/odometry/filtered"]])

print(f"{bag.rstrip('/').split('/')[-1]}: {fo[-1, 0]:.0f} s")
last = None
for t, m in D["/mavros/state"]:
    if (m.armed, m.mode) != last:
        print(f"  {t:6.1f} s  armed={m.armed} mode={m.mode}"); last = (m.armed, m.mode)

print("\nGOALS")
seen = {}
for t, m in D["/navigate_to_pose/_action/status"]:
    for s in m.status_list:
        gid = bytes(s.goal_info.goal_id.uuid).hex()[:6]
        if seen.get(gid) != s.status and s.status in (2, 4, 5, 6):
            seen[gid] = s.status
            print(f"  {t:6.1f} s  goal {gid}: {['', '', 'EXECUTING', '', 'SUCCEEDED', 'CANCELED', 'ABORTED'][s.status]}")
goals = D["/goal_pose"]
for k, (t, m) in enumerate(goals):
    gx, gy, gyaw = m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)
    tn = goals[k + 1][0] if k + 1 < len(goals) else fo[-1, 0]
    seg = fo[(fo[:, 0] >= t) & (fo[:, 0] <= tn)]
    if len(seg) < 2:
        continue
    d = np.hypot(seg[:, 1] - gx, seg[:, 2] - gy)
    dist = np.sum(np.hypot(np.diff(seg[:, 1]), np.diff(seg[:, 2])))
    print(f"  goal {k + 1} at {t:6.1f} s ({m.header.frame_id}): ({gx:+.2f}, {gy:+.2f}) yaw {math.degrees(gyaw):+.0f} | drove {dist:.1f} m, "
          f"closest {d.min():.2f} m, end {d[-1]:.2f} m, final heading error {math.degrees(wrap(seg[-1, 3] - gyaw)):+.0f} deg")

print("\nIN-PLACE TURNS (raw vx < 0.05, |wz| > 0.1, > 1 s)")
raw = np.array([(t, m.linear.x, m.angular.z) for t, m in D["/cmd_vel_nav_raw"]]) if D["/cmd_vel_nav_raw"] else np.zeros((0, 3))
out = np.array([(t, m.linear.x, m.angular.z) for t, m in D["/cmd_vel_nav"]]) if D["/cmd_vel_nav"] else np.zeros((0, 3))
if len(raw):
    turn = (np.abs(raw[:, 1]) < 0.05) & (np.abs(raw[:, 2]) > 0.1)
    s0 = None
    for i in range(len(raw)):
        if turn[i] and s0 is None:
            s0 = i
        if (not turn[i] or i == len(raw) - 1) and s0 is not None:
            ta, tb = raw[s0, 0], raw[i - 1, 0]
            if tb - ta > 1.0:
                o = out[(out[:, 0] >= ta) & (out[:, 0] <= tb)]; f = fo[(fo[:, 0] >= ta) & (fo[:, 0] <= tb)]
                print(f"  {ta:6.1f}-{tb:6.1f} s: commanded {np.median(np.abs(raw[s0:i, 2])):.2f}, after monitor "
                      f"{np.median(np.abs(o[:, 2])) if len(o) else float('nan'):.2f}, actual {np.median(np.abs(f[:, 5])):.2f} rad/s, "
                      f"turned {math.degrees(np.sum(np.diff(np.unwrap(f[:, 3])))):+.0f} deg")
            s0 = None
    r = []
    for t, x, z in raw:
        k = np.searchsorted(out[:, 0], t)
        if k < len(out) and abs(out[k, 0] - t) < 0.2:
            a, b = math.hypot(x, z), math.hypot(out[k, 1], out[k, 2]); r.append(b / a if a > 1e-3 else 1)
    r = np.array(r)
    print(f"\nCOLLISION MONITOR: unchanged {np.mean(r > 0.97):.0%}, slowed {np.mean((r > 0.05) & (r <= 0.97)):.0%}, "
          f"stopped {np.mean(r <= 0.05):.0%} of command samples")

print("\nLOG (warnings / errors, first of each)")
c = collections.Counter()
for t, m in D["/rosout"]:
    if (m.level >= 30 or "FCU" in m.msg) and "foxglove" not in m.name:
        key = (m.name, m.msg[:80])
        if c[key] == 0:
            print(f"  {t:6.1f} s [{m.name}] {m.msg[:110]}")
        c[key] += 1
