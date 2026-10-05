#!/usr/bin/env python3
"""Offline test of lidar_odom (dagger_perception), the GNSS-independent LiDAR odometry. NO ROVER.

    python3 tools/test_lidar_odom.py                 # synthetic scenes + fox3 replay
    python3 tools/test_lidar_odom.py --bag <bag>     # another drive (replay part only reports)

Runs in ROS_DOMAIN_ID 84 (set before rclpy is imported) and starts lidar_odom with its default
parameters. Clouds are published in the rslidar layout with the rover's URDF mount on
/tf_static, one at a time (lockstep: the next cloud only after the node's status for the
previous one), so nothing is dropped and the result is deterministic.
  1. synthetic: a room, then the same room seen after a known move (x, y, yaw) -> the twist;
     too few points and the rover's own frame alone -> rejected, no odometry; a corridor
     (two long walls + floor) -> degenerate along its axis, covariance inflated there;
  2. replay of detour_fox3_144415 (where /odometry/filtered stalled, Q72): every scan
     accepted, CPU, and the forward distance in 107-115 s against a direct registration of
     the two end scans (small_gicp GICP, 0.1 m voxels) -- the FCU odometry said -0.2 m there.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "84"
import argparse, array, atexit, math, signal, subprocess, sys, time
import numpy as np
import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from dagger_msgs.msg import LidarOdomStatus

BAGS = os.path.expanduser("~/dagger_bags")
ap = argparse.ArgumentParser()
ap.add_argument("--bag", default=f"{BAGS}/detour_fox3_144415")
a = ap.parse_args()

SRC = ("export ROS_DOMAIN_ID=84 && source /opt/ros/humble/setup.bash && "
       "source /home/jetson/ros2_ws/install/setup.bash")
proc = subprocess.Popen(
    ["bash", "-lc", f"{SRC} && exec ros2 run dagger_perception lidar_odom --ros-args "
     "-r cloud_in:=/t/cloud -r odometry:=/t/odom -r status:=/t/status"],
    stdout=open("/tmp/lidar_odom_test.log", "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)


@atexit.register
def stop_node():
    for sig in (signal.SIGINT, signal.SIGKILL):
        try: os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError: pass
        time.sleep(1.0)


rclpy.init(); n = Node("lidar_odom_test")
odoms, stats = {}, {}
key = lambda h: (h.stamp.sec, h.stamp.nanosec)
n.create_subscription(Odometry, "/t/odom", lambda m: odoms.__setitem__(key(m.header), m), 100)
n.create_subscription(LidarOdomStatus, "/t/status", lambda m: stats.__setitem__(key(m.header), m), 100)
cloud_pub = n.create_publisher(PointCloud2, "/t/cloud", QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1))
tf_pub = n.create_publisher(TFMessage, "/tf_static", QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1))


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
DT = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
               "formats": [np.float32] * 4 + [np.uint16, np.float64],
               "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 26})
FIELDS = [PointField(name=nm, offset=o, datatype=d, count=1) for nm, o, d in
          (("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("ring", 16, 4), ("timestamp", 18, 8))]
clock = [100.0]


def cloud(points_base, dt=0.1):
    """Points in base_footprint -> a cloud in the rslidar frame, stamp advanced by dt."""
    P = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    S = (P - T) @ RZ
    arr = np.zeros(len(S), dtype=DT); arr["x"], arr["y"], arr["z"] = S[:, 0], S[:, 1], S[:, 2]
    clock[0] += dt
    m = PointCloud2(); m.header.frame_id = "rslidar"
    m.header.stamp.sec = int(clock[0]); m.header.stamp.nanosec = int(round((clock[0] % 1) * 1e9))
    m.height, m.width, m.fields, m.point_step = 1, len(S), FIELDS, 26
    m.row_step, m.is_dense = 26 * len(S), False
    m.data = array.array("B", arr.tobytes())
    return m


def run(m, timeout=3.0):
    k = key(m.header); cloud_pub.publish(m)
    end = time.time() + timeout
    while time.time() < end and k not in stats:
        rclpy.spin_once(n, timeout_sec=0.005)
    end = time.time() + 0.05                        # odometry is published before the status
    while time.time() < end and k not in odoms:
        rclpy.spin_once(n, timeout_sec=0.005)
    return stats.pop(k, None), odoms.pop(k, None)


checks = []
def check(name, ok, detail=""):
    checks.append(bool(ok)); print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw); return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


rng = np.random.default_rng(0)
def room():
    """A 12 x 9 m room with a floor, four walls and three boxes: well constrained."""
    pts = []
    for x in np.arange(-6, 6, 0.08):
        for z in np.arange(0.1, 2.5, 0.12):
            pts += [[x, 4.5, z], [x, -4.5, z]]
    for y in np.arange(-4.5, 4.5, 0.08):
        for z in np.arange(0.1, 2.5, 0.12):
            pts += [[6.0, y, z], [-6.0, y, z]]
    for x in np.arange(-6, 6, 0.15):
        for y in np.arange(-4.5, 4.5, 0.15):
            pts.append([x, y, 0.0])
    for cx, cy in ((2.5, 1.5), (-3.0, -2.0), (3.5, -2.5)):
        for x in np.arange(cx - 0.4, cx + 0.4, 0.05):
            for z in np.arange(0, 1.0, 0.08):
                pts += [[x, cy - 0.4, z], [x, cy + 0.4, z]]
    return np.array(pts) + rng.normal(0, 0.005, (len(pts), 3))


def corridor():
    """Two 60 m walls 2 m apart along x, and a floor: nothing pins x."""
    pts = []
    for x in np.arange(-30, 30, 0.1):
        for z in np.arange(0.1, 2.4, 0.15):
            pts += [[x, 1.0, z], [x, -1.0, z]]
        for y in np.arange(-1.0, 1.0, 0.2):
            pts.append([x, y, 0.0])
    return np.array(pts) + rng.normal(0, 0.005, (len(pts), 3))


def seen_from(world, x, y, yaw):
    """World points as the rover sees them at pose (x, y, yaw), up to 25 m."""
    Tinv = np.linalg.inv(se2(x, y, yaw))
    xy = (Tinv @ np.c_[world[:, :2], np.ones(len(world))].T).T[:, :2]
    P = np.c_[xy, world[:, 2]]
    return P[np.linalg.norm(P, axis=1) < 25]


# ---- wait for the node ------------------------------------------------------------
W = room()
end = time.time() + 20
while time.time() < end:
    s, _ = run(cloud(seen_from(W, 0, 0, 0)), timeout=0.5)
    if s is not None:
        break
else:
    print("FAIL: lidar_odom did not answer -- see /tmp/lidar_odom_test.log"); sys.exit(1)

print("\n1. Synthetic scenes")
for _ in range(3):
    s, o = run(cloud(seen_from(W, 0, 0, 0)))
check("static room: accepted, twist ~0", s.accepted and o is not None and abs(o.twist.twist.linear.x) < 0.01,
      f"vx {o.twist.twist.linear.x:+.4f}" if o else "no odometry")
pose = [0.0, 0.0, 0.0]
vx_seen, wz_seen = [], []
for k in range(20):                                   # 0.3 m/s forward, turning 0.2 rad/s, 10 Hz
    pose[2] += 0.02; pose[0] += 0.03 * math.cos(pose[2]); pose[1] += 0.03 * math.sin(pose[2])
    s, o = run(cloud(seen_from(W, *pose)))
    if o: vx_seen.append(o.twist.twist.linear.x); wz_seen.append(o.twist.twist.angular.z)
check("moving 0.3 m/s, turning 0.2 rad/s: twist reproduced (median)",
      len(vx_seen) == 20 and abs(np.median(vx_seen) - 0.3) < 0.01 and abs(np.median(wz_seen) - 0.2) < 0.01,
      f"{len(vx_seen)}/20 published, vx {np.median(vx_seen):.3f}, wz {np.median(wz_seen):.3f}")
q = o.pose.pose.orientation
check("pose after 2 s matches the motion", abs(o.pose.pose.position.x - pose[0]) < 0.03 and
      abs(o.pose.pose.position.y - pose[1]) < 0.03 and abs(2 * math.atan2(q.z, q.w) - pose[2]) < 0.01,
      f"({o.pose.pose.position.x:.3f}, {o.pose.pose.position.y:.3f}) vs ({pose[0]:.3f}, {pose[1]:.3f})")
s, o = run(cloud(np.array([[3.0, 0.0, 1.0]] * 50)))
check("too few points: rejected, no odometry", not s.accepted and o is None and s.reject_reason == "too few points",
      s.reject_reason)
body = [[-0.84, y, z] for y in np.linspace(-0.45, 0.45, 40) for z in np.linspace(0.81, 1.11, 20)]
s, o = run(cloud(np.array(body)))
check("the rover's own frame alone: removed by the body box -> too few points", not s.accepted and s.points == 0,
      f"points {s.points}")
s, o = run(cloud(seen_from(W, *pose)))
check("room again after the rejections: accepted, pose continuous", s.accepted and o is not None and
      abs(o.pose.pose.position.x - pose[0]) < 0.05, f"x {o.pose.pose.position.x:.3f}" if o else "none")
C = corridor()
info, weak, cov, degen = [], [], [], []
for k in range(15):                                   # 3 s gap first: the map is rebuilt from the corridor
    s, o = run(cloud(seen_from(C, 0.03 * k, 0, 0), dt=3.0 if k == 0 else 0.1))
    if s.accepted and o is not None:
        info.append(s.info_min); weak.append(abs(s.weak_axis[0])); degen.append(s.degenerate)
        cov.append((o.twist.covariance[0], o.twist.covariance[7]))
check("corridor (after a 3 s gap -> map rebuilt): degenerate, weak axis along the corridor (x)",
      len(info) >= 12 and all(degen) and np.median(weak) > 0.9,
      f"{len(info)} published, {sum(degen)} degenerate, info_min {np.median(info):.0f}, |weak.x| {np.median(weak):.2f}")
check("... twist variance along x >> across (y)", cov and cov[-1][0] > 100 * cov[-1][1],
      f"var x {cov[-1][0]:.3g}, var y {cov[-1][1]:.3g}" if cov else "")

print(f"\n2. Replay {os.path.basename(a.bag.rstrip('/'))}")
if not os.path.isdir(a.bag):
    print(f"  [SKIP] {a.bag} not found")
else:
    try:
        import small_gicp                     # the reference registration (pip small_gicp)
    except ImportError:
        small_gicp = None
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    t0 = rd.get_metadata().starting_time.nanoseconds * 1e-9
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/rslidar_points"]))
    rows, scans = [], {}
    while rd.has_next():
        _, d, ts = rd.read_next(); tb = ts * 1e-9 - t0
        m = deserialize_message(d, PointCloud2)
        s, o = run(m)
        rows.append((tb, s, o))
        if 106.5 <= tb <= 116.5 and int(tb * 10) % 5 == 0:      # keep a few for the reference
            a_ = np.frombuffer(bytes(m.data), dtype=DT)
            P = np.stack([a_["x"], a_["y"], a_["z"]], 1).astype(np.float64); P = P[np.isfinite(P).all(1)]
            B = P @ RZ.T + T; B = B[~((B[:, 0] > -0.9) & (B[:, 0] < 0.5) & (np.abs(B[:, 1]) < 0.6))]
            scans[tb] = (B, o)
    st = [r[1] for r in rows if r[1] is not None]
    acc = sum(s.accepted for s in st); ms = np.array([s.cpu_ms for s in st])
    check("every scan answered", len(st) == len(rows), f"{len(st)}/{len(rows)}")
    check("all but the first accepted, no map reset during the drive", acc >= len(st) - 1 and
          st[-1].resets_total == st[0].resets_total,
          f"{acc}/{len(st)} accepted, {st[-1].rejected_total - st[0].rejected_total} rejected, "
          f"{st[-1].resets_total - st[0].resets_total} resets")
    check("CPU per scan (one thread): mean < 50 ms, p95 < 80 ms", ms.mean() < 50 and np.percentile(ms, 95) < 80,
          f"mean {ms.mean():.1f}, p95 {np.percentile(ms, 95):.1f}, max {ms.max():.1f} ms")
    rej = [(tb, s) for tb, s, _ in rows if s is not None and (not s.accepted or s.degenerate)]
    for tb, s in rej[:40]:
        print(f"     {tb:7.1f} s  {'accepted' if s.accepted else 'REJECTED'} {s.reject_reason:16s} inliers {s.inlier_ratio:.2f}"
              f"  it {s.iterations:2d}  info_min {s.info_min:8.0f}  points {s.points}  dt {s.dt:.2f}")
    deg = sum(s.degenerate for s in st); info = np.array([s.info_min for s in st if s.accepted and s.info_min > 0])
    print(f"     degenerate scans: {deg}; translational information min {info.min():.0f}, p5 {np.percentile(info, 5):.0f}, median {np.median(info):.0f}")
    if scans and small_gicp is None:
        print("  [SKIP] the 107-115 s reference needs the small_gicp Python package (pip install small_gicp)")
    elif scans:
        ka, kb = min(scans), max(scans)
        (Pa, oa), (Pb, ob) = scans[ka], scans[kb]
        def T_of(o):
            q = o.pose.pose.orientation; p = o.pose.pose.position; y = 2 * math.atan2(q.z, q.w)
            M = np.eye(4); M[:2, :2] = [[math.cos(y), -math.sin(y)], [math.sin(y), math.cos(y)]]; M[:3, 3] = (p.x, p.y, p.z); return M
        rel = np.linalg.inv(T_of(oa)) @ T_of(ob)
        r = small_gicp.align(Pa, Pb, rel, registration_type="GICP", downsampling_resolution=0.1,
                             max_correspondence_distance=1.0, max_iterations=50, num_threads=4)
        ref = r.T_target_source
        check(f"{ka:.1f}-{kb:.1f} s (odometry stalled there): forward distance = direct registration +- 0.10 m",
              abs(rel[0, 3] - ref[0, 3]) < 0.10, f"lidar_odom {rel[0, 3]:.2f} m, reference {ref[0, 3]:.2f} m")

print(f"\n{sum(checks)}/{len(checks)} passed")
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if all(checks) else 1)
