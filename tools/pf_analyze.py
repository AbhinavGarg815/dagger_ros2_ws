#!/usr/bin/env python3
"""Analyze a Dagger path-following test bag (recorded by tools/pf_record.sh).

    python3 tools/pf_analyze.py ~/dagger_bags/pf_20260914_150000 [--png out.png]

Reports, against the Nav2 /plan active at each instant:
  cross-track error (CTE, signed, +left), heading error vs path tangent,
  commanded w vs actual w (IMU gz), command reversals (hunting metric),
  speed tracking, goal error, and controller/bt warnings from /rosout.
The key question each run answers: is the error coming from the PLANT (actual w
doesn't follow cmd w), the CONTROLLER (cmd w itself oscillates), or the POSE
(pose jitter while cmd is smooth)?
"""
import argparse
import collections
import math
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WANT = {
    "/plan", "/odometry/filtered", "/cmd_vel_nav",
    "/mavros/setpoint_velocity/cmd_vel_unstamped", "/mavros/imu/data",
    "/mavros/gpsstatus/gps1/raw", "/mavros/state", "/lookahead_point", "/rosout",
}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    storage = "mcap" if any(f.endswith(".mcap") for f in os.listdir(path)) else "sqlite3"
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = collections.defaultdict(list)
    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic in WANT and topic in types:
            out[topic].append((t_ns * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


def cte_to_path(px, py, path_xy):
    """Signed distance (+left of path direction) and path tangent at closest point."""
    a, b = path_xy[:-1], path_xy[1:]
    d = b - a
    L2 = np.maximum((d ** 2).sum(1), 1e-12)
    t = np.clip(((px - a[:, 0]) * d[:, 0] + (py - a[:, 1]) * d[:, 1]) / L2, 0, 1)
    cx, cy = a[:, 0] + t * d[:, 0], a[:, 1] + t * d[:, 1]
    dist = np.hypot(px - cx, py - cy)
    i = int(np.argmin(dist))
    cross = d[i, 0] * (py - a[i, 1]) - d[i, 1] * (px - a[i, 0])
    return math.copysign(dist[i], cross), math.atan2(d[i, 1], d[i, 0]), i == len(d) - 1 and t[i] >= 1


def reversals(t, w, dead=0.05):
    """Count sign flips of w ignoring |w|<dead (hunting metric)."""
    s = np.sign(np.where(np.abs(w) < dead, 0, w))
    s = s[s != 0]
    return int((np.diff(s) != 0).sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--png", default=None, help="plot path (default <bag>/pf.png)")
    ap.add_argument("--from-abs", type=float, default=0.0,
                    help="ignore messages before this unix time (e.g. crop a spin off a leg)")
    a = ap.parse_args()
    b = read_bag(a.bag)
    if a.from_abs:
        b = collections.defaultdict(list, {k: [(t, m) for t, m in v if t >= a.from_abs]
                                           for k, v in b.items()})

    od = b["/odometry/filtered"]
    if not od:
        raise SystemExit("no /odometry/filtered in bag")
    t0 = od[0][0]
    ot = np.array([t - t0 for t, _ in od])
    ox = np.array([m.pose.pose.position.x for _, m in od])
    oy = np.array([m.pose.pose.position.y for _, m in od])
    oyaw = np.array([yaw_of(m.pose.pose.orientation) for _, m in od])
    spd = np.hypot(np.gradient(ox, ot), np.gradient(oy, ot)) if len(ot) > 2 else np.zeros_like(ot)

    plans = [(t - t0, np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses]))
             for t, m in b["/plan"] if len(m.poses) >= 2]
    cmd_topic = "/cmd_vel_nav" if b["/cmd_vel_nav"] else "/mavros/setpoint_velocity/cmd_vel_unstamped"
    cm = b[cmd_topic]
    ct = np.array([t - t0 for t, _ in cm])
    cv = np.array([m.linear.x for _, m in cm])
    cw = np.array([m.angular.z for _, m in cm])
    im = b["/mavros/imu/data"]
    it = np.array([t - t0 for t, _ in im])
    iw = np.array([m.angular_velocity.z for _, m in im])

    # Per-odom-sample CTE / heading error vs the plan active at that instant.
    cte = np.full_like(ot, np.nan)
    herr = np.full_like(ot, np.nan)
    if plans:
        pt = np.array([p[0] for p in plans])
        for k in range(len(ot)):
            j = np.searchsorted(pt, ot[k], side="right") - 1
            if j < 0:
                continue
            e, tan, _ = cte_to_path(ox[k], oy[k], plans[j][1])
            cte[k], herr[k] = e, wrap(oyaw[k] - tan)

    # Only score while the controller is actually driving.
    driving = np.zeros_like(ot, dtype=bool)
    if len(ct):
        ci = np.clip(np.searchsorted(ct, ot) - 1, 0, len(ct) - 1)
        driving = (np.abs(ot - ct[ci]) < 0.5) & ((np.abs(cv[ci]) > 0.02) | (np.abs(cw[ci]) > 0.02))

    print(f"bag: {a.bag}")
    print(f"duration {ot[-1]:.1f} s | odom {len(ot) / max(ot[-1], 1e-6):.1f} Hz | "
          f"cmd ({cmd_topic}) {len(ct) / max(ot[-1], 1e-6):.1f} Hz | plans {len(plans)}")
    print(f"distance driven {np.nansum(np.hypot(np.diff(ox), np.diff(oy))):.2f} m | "
          f"net ({ox[-1] - ox[0]:+.2f}, {oy[-1] - oy[0]:+.2f}) m | "
          f"yaw {math.degrees(oyaw[0]):.1f} -> {math.degrees(oyaw[-1]):.1f} deg")
    m = driving & ~np.isnan(cte)
    if m.any():
        print(f"CTE (driving): RMS {np.sqrt(np.mean(cte[m] ** 2)):.3f} m, "
              f"mean {np.mean(cte[m]):+.3f}, max |{np.max(np.abs(cte[m])):.3f}| m")
        print(f"heading err  : RMS {math.degrees(np.sqrt(np.mean(herr[m] ** 2))):.1f} deg, "
              f"mean {math.degrees(np.mean(herr[m])):+.1f} deg")
    if len(ct):
        drv = (np.abs(cv) > 0.02) | (np.abs(cw) > 0.02)
        print(f"cmd v: mean {cv[drv].mean() if drv.any() else 0:.2f} m/s | actual speed (driving) "
              f"mean {spd[driving].mean() if driving.any() else 0:.2f} m/s")
        print(f"cmd w: RMS {np.sqrt(np.mean(cw[drv] ** 2)) if drv.any() else 0:.3f} rad/s, "
              f"reversals {reversals(ct, cw)} over {ct[-1] - ct[0]:.1f} s")
        if len(it):
            iw_c = np.interp(ct, it, iw)
            if drv.sum() > 5:
                cc = np.corrcoef(cw[drv], iw_c[drv])[0, 1]
                gain = np.polyfit(cw[drv], iw_c[drv], 1)[0]
                # lag: shift of max cross-correlation between cmd w and imu gz
                dt = np.median(np.diff(ct))
                x = cw[drv] - cw[drv].mean()
                y = iw_c[drv] - iw_c[drv].mean()
                xc = np.correlate(y, x, "full")[len(x) - 1:]
                lag = np.argmax(xc[: int(3 / max(dt, 1e-3))]) * dt
                print(f"plant: actual-vs-cmd w corr {cc:.2f}, gain {gain:.2f}, lag ~{lag:.2f} s "
                      f"| imu gz reversals {reversals(it, iw, 0.03)}")
    if plans:
        goal = plans[-1][1][-1]
        print(f"final goal err {math.hypot(ox[-1] - goal[0], oy[-1] - goal[1]):.2f} m")
    la = b["/lookahead_point"]
    if la:
        lt = np.array([t - t0 for t, _ in la])
        ld = [math.hypot(m.point.x - np.interp(t, ot, ox), m.point.y - np.interp(t, ot, oy))
              if m.header.frame_id != "base_footprint" else math.hypot(m.point.x, m.point.y)
              for t, m in zip(lt, [m for _, m in la])]
        print(f"lookahead dist: median {np.median(ld):.2f} m")
    g = b["/mavros/gpsstatus/gps1/raw"]
    if g:
        fx = collections.Counter(m.fix_type for _, m in g)
        ha = np.array([m.h_acc / 1000 for _, m in g])
        print(f"GNSS fix types {dict(fx)} | h_acc median {np.median(ha):.2f} m max {ha.max():.2f} m")
    modes = [m.mode for _, m in b["/mavros/state"]]
    if modes:
        print(f"modes seen: {sorted(set(modes))}")
    logs = collections.Counter()
    for _, m in b["/rosout"]:
        if m.level >= 30 and m.name in ("controller_server", "bt_navigator", "planner_server",
                                         "behavior_server", "local_costmap.local_costmap",
                                         "global_costmap.global_costmap"):
            logs[(m.name, m.msg[:110])] += 1
    if logs:
        print("rosout WARN/ERROR (top 12):")
        for (n, msg), c in logs.most_common(12):
            print(f"  {c:4d}x [{n}] {msg}")

    # ---- plot
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(14, 10))
    A = ax[0, 0]
    for _, p in plans:
        A.plot(p[:, 0], p[:, 1], color="0.75", lw=0.8)
    sc = A.scatter(ox, oy, c=ot, s=4, cmap="viridis")
    A.plot(ox[0], oy[0], "go", ms=8, label="start")
    if plans:
        A.plot(*plans[-1][1][-1], "r*", ms=14, label="goal")
    step = max(1, len(ox) // 25)
    A.quiver(ox[::step], oy[::step], np.cos(oyaw[::step]), np.sin(oyaw[::step]),
             width=0.003, scale=25, color="k")
    A.set_aspect("equal")
    A.set_title("XY (grey=plans, colour=time, arrows=heading)")
    A.legend()
    fig.colorbar(sc, ax=A, label="t [s]")
    B = ax[0, 1]
    B.plot(ot, cte, label="CTE [m] (+left)")
    B.plot(ot, np.degrees(herr) / 50, label="heading err [deg/50]")
    B.axhline(0, color="k", lw=0.5)
    B.set_title("tracking error")
    B.legend()
    C = ax[1, 0]
    C.plot(ct, cw, label="cmd w")
    C.plot(it, iw, label="imu gz (actual)", alpha=0.7)
    C.axhline(0, color="k", lw=0.5)
    C.set_title("yaw rate: command vs actual")
    C.set_xlabel("t [s]")
    C.legend()
    D = ax[1, 1]
    D.plot(ct, cv, label="cmd v")
    D.plot(ot, spd, label="actual speed", alpha=0.7)
    D.set_title("speed: command vs actual")
    D.set_xlabel("t [s]")
    D.legend()
    for x in ax.flat:
        x.grid(alpha=0.3)
    out = a.png or os.path.join(a.bag, "pf.png")
    fig.tight_layout()
    fig.savefig(out, dpi=80)
    print(f"plot: {out}")


if __name__ == "__main__":
    main()
