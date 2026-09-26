#!/usr/bin/env python3
"""Log and score slam_toolbox replays (tools/slam_replay.sh).

  --log poses.csv                  record /slam/pose until Ctrl-C (used by slam_replay.sh)
  <bag> <label> [<label> ...]      score each replay against the bag's /odometry/filtered

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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    f.close()


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log")
    ap.add_argument("bag", nargs="?")
    ap.add_argument("labels", nargs="*")
    a = ap.parse_args()
    if a.log:
        log(a.log)
        return
    if not a.bag or not a.labels:
        ap.error("need <bag> <label> ...")
    od = read_odom(os.path.expanduser(a.bag))
    for label in a.labels:
        score(od, label)


if __name__ == "__main__":
    sys.exit(main())
