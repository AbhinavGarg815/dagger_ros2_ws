#!/usr/bin/env python3
"""Offline test of safety_scan (dagger_perception), the node behind /scan_safety. NO ROVER.

    python3 tools/test_safety_scan.py
    python3 tools/test_safety_scan.py --empty <bag> --walk <bag>   # other captures

Runs in ROS_DOMAIN_ID 78 (set before rclpy is imported, inherited by the nodes it
starts) and starts two instances:
    rover -- the live configuration: body box on, range_min 0.3;
    p2l   -- body box off, range_min 1.0: must reproduce pointcloud_to_laserscan.
Parts:
  1. synthetic clouds in the LiDAR frame (rslidar point layout, the rover's mount):
     body box, height band, range limits, nearest-per-bearing, bins, NaN, empty;
  2. equivalence: each cloud of the captures through 'p2l', against the /scan_safety
     that p2l_safety published live from that same cloud (paired by stamp);
  3. captures through 'rover': parked with nothing near -> no return within 1.6 m;
     walk-around (2026-10-02) -> the operator at all 8 spots, nothing when nobody is near.
Parts 2-3 are skipped if the bags are missing. Captures: COMMANDS.md section 8.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "78"
import argparse, array, atexit, math, signal, subprocess, sys, time
import numpy as np
import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_msgs.msg import TFMessage

BAGS = os.path.expanduser("~/dagger_bags")
ap = argparse.ArgumentParser()
ap.add_argument("--empty", default=f"{BAGS}/body_empty_132834", help="parked, nothing within 1.6 m")
ap.add_argument("--walk", default=f"{BAGS}/body_walk_132959", help="operator around the hull")
a = ap.parse_args()

SRC = ("export ROS_DOMAIN_ID=78 && source /opt/ros/humble/setup.bash && "
       "source /home/jetson/ros2_ws/install/setup.bash")
NODES = {"rover": "", "p2l": "-p body_filter:=false -p range_min:=1.0"}
procs = [subprocess.Popen(
    ["bash", "-lc", f"{SRC} && exec ros2 run dagger_perception safety_scan --ros-args "
     f"-r __node:=ss_{k} -r cloud_in:=/t/cloud -r scan:=/t/scan_{k} {extra}"],
    stdout=open(f"/tmp/safety_scan_test_{k}.log", "w"), stderr=subprocess.STDOUT,
    preexec_fn=os.setsid) for k, extra in NODES.items()]


@atexit.register
def stop_nodes():                 # also on an exception: never leave the two nodes running
    for p in procs:
        try: os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except ProcessLookupError: pass
    time.sleep(1.5)
    for p in procs:
        try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError: pass

rclpy.init(); n = Node("safety_scan_test")
scans = {k: {} for k in NODES}
be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
for k in NODES:
    n.create_subscription(LaserScan, f"/t/scan_{k}",
                          lambda m, k=k: scans[k].__setitem__((m.header.stamp.sec, m.header.stamp.nanosec), m), be)
cloud_pub = n.create_publisher(PointCloud2, "/t/cloud", QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1))
tf_pub = n.create_publisher(TFMessage, "/tf_static", QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1))

# The rover's mount (URDF): base_footprint -> base_link z 0.2 -> lidar_link (0.2925, 0, 0.92,
# yaw 1.5708) -> rslidar (identity) = LiDAR at (0.2925, 0, 1.12), rotated 90 deg.
def tf(parent, child, xyz, yaw=0.0):
    t = TransformStamped(); t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return t
tf_pub.publish(TFMessage(transforms=[tf("base_footprint", "base_link", (0, 0, 0.2)),
                                     tf("base_link", "lidar_link", (0.2925, 0, 0.92), 1.5708),
                                     tf("lidar_link", "rslidar", (0, 0, 0))]))
YAW, T = 1.5708, np.array([0.2925, 0.0, 1.12])
RZ = np.array([[math.cos(YAW), -math.sin(YAW), 0], [math.sin(YAW), math.cos(YAW), 0], [0, 0, 1]])

# rslidar's layout: x y z intensity (float32), ring (uint16), timestamp (float64) = 26 bytes
DT = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
               "formats": [np.float32] * 4 + [np.uint16, np.float64],
               "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 26})
FIELDS = [PointField(name=nm, offset=o, datatype=d, count=1) for nm, o, d in
          (("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("ring", 16, 4), ("timestamp", 18, 8))]
stamp_s = [1000]


def cloud(points_base):
    """Points given in base_footprint -> a cloud in the rslidar frame."""
    P = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    S = (P - T) @ RZ                       # = RZ^T (p - T) per row
    arr = np.zeros(len(S), dtype=DT)
    arr["x"], arr["y"], arr["z"] = S[:, 0], S[:, 1], S[:, 2]
    m = PointCloud2(); stamp_s[0] += 1
    m.header.frame_id, m.header.stamp.sec = "rslidar", stamp_s[0]
    m.height, m.width, m.fields, m.point_step = 1, len(S), FIELDS, 26
    m.row_step, m.is_dense = 26 * len(S), False
    m.data = array.array("B", arr.tobytes())
    return m


def run(m, timeout=2.0):
    """Publish one cloud, wait for both nodes' scan of it (lockstep: nothing dropped)."""
    key = (m.header.stamp.sec, m.header.stamp.nanosec)
    cloud_pub.publish(m)
    end = time.time() + timeout
    while time.time() < end and not all(key in scans[k] for k in NODES):
        rclpy.spin_once(n, timeout_sec=0.005)
    return {k: scans[k].pop(key, None) for k in NODES}


def xy(s):
    r = np.asarray(s.ranges, dtype=np.float64)
    ang = s.angle_min + np.arange(len(r)) * np.float64(s.angle_increment)
    ok = np.isfinite(r)
    return r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]), r[ok]


def bin_of(s, bearing):
    return int((bearing - np.float32(s.angle_min)) / np.float32(s.angle_increment))


checks = []
def check(name, ok, detail=""):
    checks.append((name, bool(ok)))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


# ---- wait for both nodes (they publish only once /tf_static resolves) ----------------
end = time.time() + 20
while time.time() < end:
    r = run(cloud([[5.0, 0.0, 1.0]]), timeout=0.5)
    if all(r.values()):
        break
else:
    print("FAIL: safety_scan did not answer -- see /tmp/safety_scan_test_*.log")
    sys.exit(1)

print("\n1. Synthetic clouds")
r = run(cloud(np.zeros((0, 3))))
s = r["rover"]
check("empty cloud: 723 bins, all inf, frame base_footprint, stamp kept",
      s and len(s.ranges) == 723 and np.all(np.isinf(s.ranges)) and s.header.frame_id == "base_footprint"
      and s.header.stamp.sec == stamp_s[0])
check("scan fields: range 0.3-20 (rover) / 1.0-20 (p2l), angles as p2l_safety",
      abs(s.range_min - 0.3) < 1e-6 and abs(r["p2l"].range_min - 1.0) < 1e-6 and abs(s.range_max - 20) < 1e-6
      and abs(s.angle_min + 3.14159) < 1e-5 and abs(s.angle_increment - 0.0087) < 1e-6)

# The rover itself (Q65): rear rail, side rails, the two posts -- every height of the frame.
zs = np.linspace(0.81, 1.11, 7)
body = ([[-0.84, y, z] for y in np.linspace(-0.45, 0.45, 19) for z in zs] +
        [[x, sy * 0.51, z] for x in np.linspace(-0.80, 0.0, 17) for sy in (-1, 1) for z in zs] +
        [[x, sy * 0.30, z] for x in (0.31, 0.36, 0.41) for sy in (-1, 1) for z in (1.05, 1.08, 1.11)])
r = run(cloud(body))
check("the rover's own frame and posts: no return (rover)", np.all(np.isinf(r["rover"].ranges)))
check("... and none with p2l either (all inside range_min 1.0)", np.all(np.isinf(r["p2l"].ranges)))

# Just outside the box: front deck, beside the side rail, behind the rear rail.
pts = [[0.52, 0.0, 1.0], [-0.40, 0.62, 1.0], [-0.93, 0.05, 1.0]]
r = run(cloud(pts)); s = r["rover"]
got = [s.ranges[bin_of(s, math.atan2(p[1], p[0]))] for p in pts]
check("just outside the body box is kept (front deck 0.52, side 0.62, rear -0.93)",
      all(abs(g - math.hypot(p[0], p[1])) < 1e-3 for g, p in zip(got, pts)),
      ", ".join(f"{g:.3f}" for g in got))
check("... and the same three are invisible to p2l (inside 1.0 m)", np.all(np.isinf(r["p2l"].ranges)))

# A person beside the left side, the case p2l could not see (walk capture: 0 rays there).
th = np.linspace(-math.pi, 0, 40)            # the half of a 0.2 m cylinder facing the rover
person = [[0.2 * math.cos(t), 0.80 + 0.2 * math.sin(t), z] for t in th for z in np.linspace(0.9, 1.4, 11)]
r = run(cloud(person))
nr = int(np.sum(xy(r["rover"])[2] <= 1.03)); npl = int(np.sum(xy(r["p2l"])[2] <= 1.03))
check("person beside the left side, 0.6 m from centre: rays within 1.03 m", nr >= 10 and npl == 0,
      f"rover {nr}, p2l {npl}")

band = [[2.0, 0.5, 0.29], [2.0, -0.5, 0.31], [-2.0, 0.5, 1.59], [-2.0, -0.5, 1.61]]
r = run(cloud(band)); s = r["rover"]
got = [math.isfinite(s.ranges[bin_of(s, math.atan2(p[1], p[0]))]) for p in band]
check("height band: z 0.29 and 1.61 dropped, 0.31 and 1.59 kept", got == [False, True, True, False])

r = run(cloud([[0.0, -21.0, 1.0], [0.95, 0.3, 1.0], [1.05, -0.3, 1.0]]))
s, p = r["rover"], r["p2l"]
check("range: beyond 20 m dropped; p2l drops 0.99 m and keeps 1.09 m (range_min 1.0)",
      np.sum(np.isfinite(s.ranges)) == 2 and np.sum(np.isfinite(p.ranges)) == 1
      and math.isfinite(p.ranges[bin_of(p, math.atan2(-0.3, 1.05))]))

b = 0.3
r = run(cloud([[3.0 * math.cos(b), 3.0 * math.sin(b), 1.0], [2.0 * math.cos(b), 2.0 * math.sin(b), 0.5]]))
s = r["rover"]
check("nearest return per bearing wins (2.0 over 3.0)", abs(s.ranges[bin_of(s, b)] - 2.0) < 1e-3)

r = run(cloud([[0.0, 2.0, 1.0]])); s = r["rover"]
i = int(np.argmin(s.ranges))
check("bin index as p2l: +90 deg lands in bin int((pi/2 + 3.14159) / 0.0087)",
      i == int((math.pi / 2 + np.float32(3.14159)) / np.float32(0.0087)), f"bin {i}")

m = cloud([[2.0, 1.0, 1.0], [3.0, -1.0, 1.0]])
d = np.frombuffer(bytes(m.data), dtype=DT).copy(); d["x"][0] = np.nan
m.data = array.array("B", d.tobytes())
r = run(m)
check("NaN point ignored, the other kept", r["rover"] and np.sum(np.isfinite(r["rover"].ranges)) == 1)


def bag_clouds(path):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/rslidar_points"]))
    while rd.has_next():
        yield deserialize_message(rd.read_next()[1], PointCloud2)


def bag_scans(path):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/scan_safety"]))
    out = {}
    while rd.has_next():
        s = deserialize_message(rd.read_next()[1], LaserScan)
        out[(s.header.stamp.sec, s.header.stamp.nanosec)] = s
    return out


have = {k: os.path.isdir(p) for k, p in (("empty", a.empty), ("walk", a.walk))}
print("\n2. Equivalence with pointcloud_to_laserscan (p2l_safety, live)")
results = {}                      # bag -> list of (t, rover scan, recorded scan)
agree, ndiff, missing = [], 0, 0
for k, path in (("empty", a.empty), ("walk", a.walk)):
    if not have[k]:
        print(f"  [SKIP] {path} not found"); continue
    rec = bag_scans(path); rows = []; t0 = None
    for m in bag_clouds(path):
        key = (m.header.stamp.sec, m.header.stamp.nanosec)
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec; t0 = t if t0 is None else t0
        r = run(m)
        if r["rover"] is None or r["p2l"] is None:
            missing += 1; continue
        rows.append((t - t0, r["rover"], rec.get(key)))
        if key in rec:
            x = np.asarray(r["p2l"].ranges); y = np.asarray(rec[key].ranges)
            same = (np.isinf(x) & np.isinf(y)) | (np.abs(np.where(np.isfinite(x), x, 0) - np.where(np.isfinite(y), y, 0)) < 1e-6)
            agree.append(same.mean()); ndiff += int((~same).sum())
    results[k] = rows
if agree:
    agree = np.array(agree)
    check("every cloud answered by both nodes", missing == 0, f"{missing} missing")
    check("p2l config reproduces the recorded /scan_safety bin for bin",
          agree.min() == 1.0,
          f"{len(agree)} scans, {np.mean(agree == 1.0):.1%} identical, worst {agree.min():.2%}, {ndiff} bins differ")

print("\n3. The captures through the rover configuration")
if "empty" in results:
    near = [int(np.sum(xy(s)[2] < 1.6)) for _, s, _ in results["empty"]]
    check("parked, nothing near: no return within 1.6 m in any scan", max(near) == 0,
          f"{len(near)} scans, max {max(near)}")
if "walk" in results:
    rows = results["walk"]
    if os.path.basename(a.walk.rstrip("/")) == "body_walk_132959":
        spots = [("nobody", 0, 14), ("front centre", 17, 25), ("left, front", 29, 36), ("left, middle", 39, 45),
                 ("left, rear", 48, 53), ("rear centre", 56, 64), ("right, rear", 68, 75),
                 ("right, middle", 78, 84), ("right, front", 87, 93), ("nobody", 102, 118)]
        print(f"     {'spot':14s} rays within 1.03 m: today (recorded)  new")
        for name, t0, t1 in spots:
            w = [(s, rec) for t, s, rec in rows if t0 <= t < t1]
            new = np.median([np.sum(xy(s)[2] <= 1.03) for s, _ in w])
            old = np.median([np.sum(xy(rec)[2] <= 1.03) for _, rec in w if rec is not None])
            print(f"     {name:14s} {old:30.0f}  {new:5.0f}")
            if name == "nobody":
                mx = max(int(np.sum(xy(s)[2] <= 1.03)) for s, _ in w)
                check(f"nobody near ({t0}-{t1} s): no ray within 1.03 m", mx == 0, f"max {mx}")
            else:
                check(f"operator at {name}: seen inside the corner sweep", new >= 30, f"median {new:.0f} rays")
    else:
        print("     rays within 1.03 m, per second:")
        for sec in range(int(rows[-1][0]) + 1):
            w = [np.sum(xy(s)[2] <= 1.03) for t, s, _ in rows if sec <= t < sec + 1]
            if w:
                print(f"     {sec:4d} s  {np.median(w):5.0f}")

print(f"\n{sum(ok for _, ok in checks)}/{len(checks)} passed")
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if all(ok for _, ok in checks) else 1)
