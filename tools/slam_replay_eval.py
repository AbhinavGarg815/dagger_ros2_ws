#!/usr/bin/env python3
"""Log and score slam_toolbox replays (tools/slam_replay.sh).

  --log poses.csv                  record /slam/pose until Ctrl-C (used by slam_replay.sh)
  <bag> <label> [<label> ...]      score each replay against the bag's /odometry/filtered
  --loc <label> ... [<bag>]        score localization replays (+ map->odom corrections with the bag)

Scores, per run: pose count; longest gap between poses and what the rover did
in it (a pivot with no pose = the lane2/lane3 turn error, Q42); SLAM-minus-odom
yaw -- its spread and its biggest pose-to-pose jump (a snap = SLAM correcting a
bad match). A healthy run has short gaps and no jumps above a few degrees.
"""
import argparse
import bisect
import csv
import math
import os
import sys


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def log(path):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rclpy.init()
    node = Node("slam_replay_logger",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["stamp", "x", "y", "yaw"])

    def cb(m):
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        w.writerow([f"{s:.3f}", m.pose.pose.position.x, m.pose.pose.position.y,
                    yaw_of(m.pose.pose.orientation)])
        f.flush()

    node.create_subscription(PoseWithCovarianceStamped, "/slam/pose", cb, 50)

    # localization replays: also log GNSS converted into the map (slam_geo_init)
    from geometry_msgs.msg import PoseStamped
    g = open(path.replace("poses.csv", "gnss.csv"), "w", newline="")
    gw = csv.writer(g)
    gw.writerow(["stamp", "x", "y", "yaw"])

    def gcb(m):
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        gw.writerow([f"{s:.3f}", m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)])
        g.flush()

    node.create_subscription(PoseStamped, "/slam/gnss_pose", gcb, 50)

    from std_msgs.msg import Float32
    h = open(path.replace("poses.csv", "score.csv"), "w", newline="")
    hw = csv.writer(h)
    hw.writerow(["stamp", "score"])

    def hcb(m):
        hw.writerow([f"{node.get_clock().now().nanoseconds * 1e-9:.3f}", f"{m.data:.4f}"])
        h.flush()

    node.create_subscription(Float32, "/slam/match_score", hcb, 50)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    f.close()
    g.close()
    h.close()


def read_odom(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/odometry/filtered"]))
    od = []
    while r.has_next():
        _, raw, _ = r.read_next()
        m = deserialize_message(raw, Odometry)
        od.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                   m.pose.pose.position.x, m.pose.pose.position.y,
                   yaw_of(m.pose.pose.orientation)))
    return od


def score(od, label):
    base = os.path.expanduser(f"~/dagger_bags/replay/{label}")
    with open(os.path.join(base, "poses.csv")) as f:
        sl = [tuple(float(v) for v in row) for row in list(csv.reader(f))[1:]]
    cpu = open(os.path.join(base, "cpu.txt")).read().strip() if os.path.exists(
        os.path.join(base, "cpu.txt")) else "cpu ?"
    if len(sl) < 2:
        print(f"{label:>10}: {len(sl)} poses -- nothing to score")
        return
    ts = [o[0] for o in od]

    def at(t):
        i = bisect.bisect_left(ts, t)
        return od[min(i, len(od) - 1)]

    def seg(a, b):
        s = [o for o in od if a <= o[0] <= b]
        if len(s) < 2:
            return 0.0, 0.0
        rot = sum(math.remainder(q[3] - p[3], 2 * math.pi) for p, q in zip(s, s[1:]))
        return math.hypot(s[-1][1] - s[0][1], s[-1][2] - s[0][2]), math.degrees(rot)

    dy = [math.degrees(math.remainder(p[3] - at(p[0])[3], 2 * math.pi)) for p in sl]
    jumps = [abs(math.remainder(math.radians(b - a), 2 * math.pi)) for a, b in zip(dy, dy[1:])]
    gaps = sorted(((b[0] - a[0], a[0], b[0]) for a, b in zip(sl, sl[1:])), reverse=True)[:3]
    ji = max(range(len(jumps)), key=jumps.__getitem__)
    print(f"{label:>10}: {len(sl)} poses | yaw(slam-odom) {min(dy):+.1f}..{max(dy):+.1f} deg, "
          f"biggest jump {math.degrees(jumps[ji]):.1f} deg at t={sl[ji + 1][0] - od[0][0]:.1f}s | {cpu}")
    for g, a, b in gaps:
        net, rot = seg(a, b)
        print(f"{'':>12}gap {g:4.1f}s at t={a - od[0][0]:5.1f}s: moved {net:.2f} m, turned {rot:+.0f} deg")


def read_odom_full(bag):
    """(stamp, x, y, yaw, v, wz) from /odometry/filtered, header stamps."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/odometry/filtered"]))
    od = []
    while r.has_next():
        _, raw, _ = r.read_next()
        m = deserialize_message(raw, Odometry)
        od.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                   m.pose.pose.position.x, m.pose.pose.position.y,
                   yaw_of(m.pose.pose.orientation),
                   m.twist.twist.linear.x, m.twist.twist.angular.z))
    return od


def corrections(sl, od):
    """How much map->odom moved at each SLAM update, split by what the rover did.
    Parked should be ~0; driving/pivoting show how much SLAM overrides odom --
    along a plain corridor that is SLAM sliding along the lane (loc1, Q44)."""
    ts = [o[0] for o in od]

    def at(t):
        i = min(max(bisect.bisect_left(ts, t), 1), len(od) - 1)
        a, b = od[i - 1], od[i]
        f = (t - a[0]) / max(b[0] - a[0], 1e-9)
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]),
                a[3] + f * math.remainder(b[3] - a[3], 2 * math.pi), b[4], b[5])

    def m2o(s, o):
        yaw = math.remainder(s[3] - o[2], 2 * math.pi)
        c, n = math.cos(yaw), math.sin(yaw)
        return s[1] - (c * o[0] - n * o[1]), s[2] - (n * o[0] + c * o[1])

    bins = {}
    for p, q in zip(sl, sl[1:]):
        a, b = at(p[0]), at(q[0])
        A, B = m2o(p, a), m2o(q, b)
        kind = ("pivot" if abs(b[4]) > 0.15 and abs(b[3]) < 0.15
                else "driving" if abs(b[3]) >= 0.15 else "parked")
        bins.setdefault(kind, []).append(math.hypot(B[0] - A[0], B[1] - A[1]))
    return bins


def score_loc(label, od=None):
    """Localization replay: SLAM pose vs GNSS-in-map after the seed, and jumps."""
    base = os.path.expanduser(f"~/dagger_bags/replay/{label}")

    def rows(name):
        with open(os.path.join(base, name)) as f:
            return [tuple(float(v) for v in r) for r in list(csv.reader(f))[1:]]
    sl, gn = rows("poses.csv"), rows("gnss.csv")
    geo = open(os.path.join(base, "geo.log")).read()
    seeded, localized = "seeded" in geo, "localized" in geo
    if len(sl) < 2 or not gn:
        print(f"{label:>12}: {len(sl)} SLAM poses, {len(gn)} GNSS -- nothing to score "
              f"(seeded={seeded})")
        return
    gts = [g[0] for g in gn]
    d = []
    for p in sl:
        i = bisect.bisect_left(gts, p[0])
        j = min((k for k in (i - 1, i) if 0 <= k < len(gn)), key=lambda k: abs(gts[k] - p[0]))
        if abs(gts[j] - p[0]) < 0.5:
            d.append(math.hypot(p[1] - gn[j][1], p[2] - gn[j][2]))
    steps = [(math.hypot(b[1] - a[1], b[2] - a[2]), b[0] - a[0], b[0] - sl[0][0])
             for a, b in zip(sl, sl[1:])]
    big = max(steps)
    ds = sorted(d)
    sc = [r[1] for r in rows("score.csv")] if os.path.exists(os.path.join(base, "score.csv")) else []
    if sc:
        ss = sorted(sc)
        print(f"{'':>14}match score (m): median {ss[len(ss) // 2]:.3f}, max {ss[-1]:.3f}, "
              f"{sum(v > 0.35 for v in sc)}/{len(sc)} checks >= 0.35")
    print(f"{label:>12}: {len(sl)} SLAM poses, seeded={seeded} localized={localized} | "
          f"SLAM-GNSS median {ds[len(ds) // 2]:.2f} m, p90 {ds[int(0.9 * (len(ds) - 1))]:.2f} m, "
          f"max {ds[-1]:.2f} m | biggest pose step {big[0]:.2f} m in {big[1]:.1f} s at +{big[2]:.0f} s")
    if od:
        for kind, v in sorted(corrections(sl, od).items()):
            v = sorted(v)
            print(f"{'':>14}map->odom correction, {kind:7s} n {len(v):3d}: median {v[len(v) // 2]:.3f}, "
                  f"p90 {v[int(0.9 * (len(v) - 1))]:.3f}, max {v[-1]:.2f} m, >0.2 m: {sum(x > 0.2 for x in v)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log")
    ap.add_argument("--loc", nargs="+", metavar="LABEL", help="score localization replays")
    ap.add_argument("bag", nargs="?")
    ap.add_argument("labels", nargs="*")
    a = ap.parse_args()
    if a.log:
        log(a.log)
        return
    if a.loc:
        od = read_odom_full(os.path.expanduser(a.bag)) if a.bag else None
        for label in a.loc:
            score_loc(label, od)
        return
    if not a.bag or not a.labels:
        ap.error("need <bag> <label> ...")
    od = read_odom(os.path.expanduser(a.bag))
    for label in a.labels:
        score(od, label)


if __name__ == "__main__":
    sys.exit(main())
