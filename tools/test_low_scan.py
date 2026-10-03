#!/usr/bin/env python3
"""Offline test of low_scan (dagger_perception) -- the low-obstacle memory's source. NO ROVER.

    python3 tools/test_low_scan.py                         # synthetic + replay of the 10-03 low-box run
    python3 tools/test_low_scan.py --replay <detour bag>   # another approach recording

Runs in ROS_DOMAIN_ID 80 (set before rclpy is imported). Two parts:
  1. synthetic Patchwork++-style clouds: a static box is confirmed once hit in >= 3 scans
     spanning >= 1.0 s (the 11th scan at 10 Hz), also
     in front of a distant wall and while the rover drives; a person (feet under a torso
     one or two bearings away) never; a low-only walker at 1.4 m/s never; nothing without
     odometry; empty bearings clear (inf), tall-only bearings do not (NaN);
  2. replay of a detour_record.sh bag (/patchworkpp/nonground + /odometry/filtered):
     /scan_low feeds a model of the costmap's low_obstacle_layer (0.1 m grid like the
     global costmap; marks finite returns < 8 m, clears along rays only beyond 4 m,
     clears under the footprint). For the 10-03 low-box run the box (near face 7.24 m)
     must be remembered from its first confirmation until the rover reaches it -- the
     normal layer lost it 3.8 m out and Nav2 drove over it (report Q68).
"""
import os
os.environ["ROS_DOMAIN_ID"] = "80"
import argparse, array, atexit, math, signal, subprocess, sys, time
import numpy as np
import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_msgs.msg import TFMessage

ap = argparse.ArgumentParser()
ap.add_argument("--replay", default=os.path.expanduser("~/dagger_bags/detour_low_114923"))
ap.add_argument("--params", default="", help='extra low_scan parameters, e.g. "-p min_span:=1.0"')
ap.add_argument("--replay-only", action="store_true", help="skip the synthetic scenes")
a = ap.parse_args()

SRC = ("export ROS_DOMAIN_ID=80 && source /opt/ros/humble/setup.bash && "
       "source /home/jetson/ros2_ws/install/setup.bash")
proc = subprocess.Popen(
    ["bash", "-lc", f"{SRC} && exec ros2 run dagger_perception low_scan --ros-args -r __node:=ls_test "
     f"-r cloud_in:=/t/cloud -r odom:=/t/odom -r scan:=/t/scan_low {a.params}"],
    stdout=open("/tmp/low_scan_test.log", "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)


@atexit.register
def stop_node():
    for sig in (signal.SIGINT, signal.SIGKILL):
        try: os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError: pass
        time.sleep(1.0)


rclpy.init(); n = Node("low_scan_test")
scans = {}
n.create_subscription(LaserScan, "/t/scan_low",
                      lambda m: scans.__setitem__((m.header.stamp.sec, m.header.stamp.nanosec), m),
                      QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50))
cloud_pub = n.create_publisher(PointCloud2, "/t/cloud", QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1))
odom_pub = n.create_publisher(Odometry, "/t/odom", QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10))
tf_pub = n.create_publisher(TFMessage, "/tf_static", QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1))


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
FIELDS = [PointField(name=nm, offset=o, datatype=7, count=1) for nm, o in (("x", 0), ("y", 4), ("z", 8))]
clock = [1000.0]
pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}


def send_odom(stamp=None):
    o = Odometry(); o.header.frame_id = "odom"
    o.pose.pose.position.x, o.pose.pose.position.y = pose["x"], pose["y"]
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(pose["yaw"] / 2), math.cos(pose["yaw"] / 2)
    odom_pub.publish(o)


def cloud(points_base, dt=0.1):
    """Points in base_footprint -> a Patchwork++-style cloud (x y z float32) in rslidar."""
    P = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    S = ((P - T) @ RZ).astype(np.float32)
    m = PointCloud2(); clock[0] += dt
    m.header.frame_id = "rslidar"; m.header.stamp.sec = int(clock[0]); m.header.stamp.nanosec = int((clock[0] % 1) * 1e9)
    m.height, m.width, m.fields, m.point_step = 1, len(S), FIELDS, 16
    m.row_step, m.is_dense = 16 * len(S), True
    buf = np.zeros((len(S), 4), dtype=np.float32); buf[:, :3] = S
    m.data = array.array("B", buf.tobytes())
    return m


def run(m, with_odom=True, timeout=2.0):
    if with_odom:
        send_odom()
    key = (m.header.stamp.sec, m.header.stamp.nanosec)
    cloud_pub.publish(m)
    end = time.time() + timeout
    while time.time() < end and key not in scans:
        rclpy.spin_once(n, timeout_sec=0.005)
    return scans.pop(key, None)


def bins_of(s, pts_base):
    out = set()
    for x, y, _ in pts_base:
        out.add(int((math.atan2(y, x) - np.float32(s.angle_min)) / np.float32(s.angle_increment)))
    return sorted(out)


checks = []
def check(name, ok, detail=""):
    checks.append(bool(ok)); print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def box(cx, cy, w=0.5, h=0.25, n=12):
    return [[cx, cy + dy, z] for dy in np.linspace(-w / 2, w / 2, n) for z in np.linspace(0.13, h, 4)]


def state(s, bins):
    r = np.asarray(s.ranges)[bins]
    return "confirmed" if np.any(np.isfinite(r)) else ("nan" if np.all(np.isnan(r)) else "inf")


# wait for the node (it publishes once /tf_static resolves)
end = time.time() + 20
while time.time() < end and run(cloud([]), timeout=0.5) is None:
    pass

print("\n1. Synthetic scenes")
if a.replay_only:
    print("  [SKIP] --replay-only")
else:

    pose.update(x=0.0, y=0.0, yaw=0.0)
    B = box(5.0, 0.0)
    seq = [run(cloud(B)) for _ in range(13)]
    bins = bins_of(seq[0], B)
    st = [state(s, bins) for s in seq]
    check("static box at 5 m: NaN for 10 scans (0.9 s), confirmed from the 11th (1.0 s)",
          st[:10] == ["nan"] * 10 and st[10:] == ["confirmed"] * 3, f"{st.count('nan')} nan, then {st.count('confirmed')} confirmed")
    r = np.asarray(seq[-1].ranges)[bins]; r = r[np.isfinite(r)]
    check("... at its range", len(r) and abs(np.min(r) - (5.0 - 0.0)) < 0.05, f"{np.min(r):.3f} m" if len(r) else "")

    s = run(cloud([]))
    check("empty cloud: all bearings inf (clear beyond 4 m)", np.all(np.isinf(s.ranges)))

    wall = [[9.0, y, z] for y in np.linspace(-1.5, 1.5, 60) for z in (0.5, 1.0, 1.4)]
    for _ in range(4):
        s = run(cloud(B + wall))
    check("box in front of a wall 4 m behind it: still confirmed", state(s, bins) == "confirmed")
    wb = [b for b in bins_of(s, wall) if b not in bins]
    check("... and the wall's own bearings are NaN (tall only: no clearing)", np.all(np.isnan(np.asarray(s.ranges)[wb])))

    # a person: feet (low) and a torso offset by ~1.5 bearings (0.013 rad at 4 m ~ 5 cm)
    pose.update(x=50.0)                                    # new place: no old hits nearby
    feet = [[4.0, 1.5 + dy, z] for dy in np.linspace(-0.08, 0.08, 5) for z in (0.15, 0.25, 0.33)]
    ang = math.atan2(1.5, 4.0) + 1.5 * 0.0087; rr = math.hypot(4.0, 1.5) + 0.1
    torso = [[rr * math.cos(ang + d), rr * math.sin(ang + d), z] for d in np.linspace(-0.01, 0.01, 4) for z in (0.8, 1.1, 1.4)]
    for _ in range(14):
        s = run(cloud(feet + torso))
    fb = bins_of(s, feet)
    check("person standing still (feet + torso 1-2 bearings away): never confirmed", state(s, fb) == "nan", state(s, fb))

    # a person crouching: knees/feet 0.3 m beside the torso -- 4+ bearings away at 4 m
    # (live walk-by 2026-10-03: such feet escaped a +-2-bearing check and were remembered)
    pose.update(x=75.0)
    feet2 = [[4.0, 1.8 + dy, z] for dy in np.linspace(-0.06, 0.06, 4) for z in (0.15, 0.25, 0.33)]
    torso2 = [[4.05, 1.5 + dy, z] for dy in np.linspace(-0.15, 0.15, 8) for z in (0.6, 0.9)]
    for _ in range(14):
        s = run(cloud(feet2 + torso2))
    check("person crouching, feet 0.3 m beside the body (static): never confirmed", state(s, bins_of(s, feet2)) == "nan",
          state(s, bins_of(s, feet2)))

    # low-only walker (worst case: no torso) crossing at 1.4 m/s
    pose.update(x=100.0)
    sts = []
    for k in range(20):
        y = -1.5 + 1.4 * 0.1 * k
        w = [[5.0, y + dy, z] for dy in np.linspace(-0.06, 0.06, 4) for z in (0.15, 0.3)]
        s = run(cloud(w)); sts.append(state(s, bins_of(s, w)))
    check("low-only walker at 1.4 m/s: never confirmed", "confirmed" not in sts, f"{sts.count('confirmed')} of 20")

    # rover driving at 0.4 m/s toward a static box (odom moves, the box stays in its odom cell)
    pose.update(x=200.0, y=0.0, yaw=0.0)
    bx = 207.0; sts = []
    for k in range(14):
        pose["x"] = 200.0 + 0.04 * k
        b = box(bx - pose["x"], 0.0)
        s = run(cloud(b)); sts.append(state(s, bins_of(s, b)))
    check("rover driving at 0.4 m/s toward a static box: confirmed from the 11th scan",
          sts[:10] == ["nan"] * 10 and all(x == "confirmed" for x in sts[10:]), f"{sts.count('nan')} nan, then {sts.count('confirmed')} confirmed")

    # no odometry: nothing confirmed
    proc_quiet = True
    pose.update(x=300.0)
    for _ in range(2):
        s = run(cloud(B), with_odom=False)
    time.sleep(0.7)                                          # older than odom_timeout (0.5 s)
    sts = [state(run(cloud(B), with_odom=False), bins) for _ in range(14)]
    check("no fresh odometry: never confirmed", "confirmed" not in sts, " ".join(sts))


# ---- 2. replay -----------------------------------------------------------------------
print(f"\n2. Replay {a.replay}")
if not os.path.isdir(a.replay):
    print("  [SKIP] bag not found")
else:
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=a.replay, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    have_cmd = any(t.topic_metadata.name == "/cmd_vel_nav_raw" and t.message_count > 0
                   for t in rd.get_metadata().topics_with_message_count)
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/patchworkpp/nonground", "/odometry/filtered", "/cmd_vel_nav_raw"]))
    RES, OBS_MAX, RT_MIN = 0.1, 8.0, 4.0
    # Where the box can show up in the start frame. Its top edge read 7.24 m from 7.2 m away;
    # the lowest beam saw its face at 6.93 m from 3.7 m (another part of the box + odometry
    # drift), and after the track ran over its corner it was re-seen at 6.63 m from behind.
    BOX_F = (6.4, 8.0)
    grid = {}                                         # remembered (marked) cells
    swept = []                                        # remembered cells the rover then drove over
    ever = {}                                         # every cell ever remembered -> first mark time
    traj = []                                         # (t, x, y) of the rover
    first = None; rows = []; t_start = None; odom_msg = None; start = None
    while rd.has_next():
        topic, data, ts = rd.read_next(); ts *= 1e-9
        if topic == "/cmd_vel_nav_raw":
            t_start = ts if t_start is None else t_start; continue
        if topic == "/odometry/filtered":
            odom_msg = deserialize_message(data, Odometry); continue
        if odom_msg is None:
            continue
        p = odom_msg.pose.pose; yaw = math.atan2(2 * p.orientation.w * p.orientation.z, 1 - 2 * p.orientation.z ** 2)
        pose.update(x=p.position.x, y=p.position.y, yaw=yaw)
        m = deserialize_message(data, PointCloud2)
        s = run(m)
        if s is None:
            continue
        if t_start is None and not have_cmd:
            t_start = ts                              # a drive without Nav2 (no commands recorded)
        if t_start is not None and start is None:
            start = (pose["x"], pose["y"], yaw)
        # model of the low_obstacle_layer
        r = np.asarray(s.ranges); ang = s.angle_min + np.arange(len(r)) * s.angle_increment + yaw
        c, sn = math.cos(yaw), math.sin(yaw)
        for key in list(grid):                       # clear: cells on a ray beyond 4 m, before its end
            gx, gy = (key[0] + 0.5) * RES - pose["x"], (key[1] + 0.5) * RES - pose["y"]
            d = math.hypot(gx, gy); bearing = math.atan2(gy, gx) - yaw
            if abs(gx * c + gy * sn) <= 0.85 and abs(-gx * sn + gy * c) <= 0.58:
                swept.append((key, grid.pop(key))); continue   # footprint clearing: driven over
            if d < RT_MIN:
                continue
            half = math.atan2(RES / 2, d)
            for bb in {int((bearing - half - s.angle_min) / s.angle_increment), int((bearing + half - s.angle_min) / s.angle_increment)}:
                if 0 <= bb < len(r) and (np.isinf(r[bb]) or (np.isfinite(r[bb]) and d < r[bb] - RES)):
                    grid.pop(key, None); break
        for bb in np.where(np.isfinite(r) & (r < OBS_MAX))[0]:   # mark
            wx, wy = pose["x"] + r[bb] * math.cos(ang[bb]), pose["y"] + r[bb] * math.sin(ang[bb])
            key = (int(math.floor(wx / RES)), int(math.floor(wy / RES)))
            grid.setdefault(key, (ts, pose["x"], pose["y"], r[bb])); ever.setdefault(key, ts)
        traj.append((ts, pose["x"], pose["y"]))
        if start is not None:
            def loc(x, y):
                dx, dy = x - start[0], y - start[1]
                return dx * math.cos(start[2]) + dy * math.sin(start[2]), -dx * math.sin(start[2]) + dy * math.cos(start[2])
            along, _ = loc(pose["x"], pose["y"])
            boxcells = [k for k in grid if (lambda f, l: BOX_F[0] < f < BOX_F[1] and -0.55 < l < 0.30)(*loc((k[0] + 0.5) * RES, (k[1] + 0.5) * RES))]
            rows.append((ts - t_start, along, len(boxcells)))
    R = np.array(rows)
    if len(R):
        seen = R[R[:, 2] > 0]
        first_t = seen[0, 0] if len(seen) else None
        # the rover reaches the box's near face when its front bumper (centre + 0.85) passes 7.24 m
        reach = R[R[:, 1] + 0.85 >= 7.24]
        t_reach = reach[0, 0] if len(reach) else R[-1, 0]
        held = R[(R[:, 0] >= (first_t or 0)) & (R[:, 0] <= t_reach)]
        print(f"     box first remembered at {first_t:.1f} s ({R[R[:,0] == first_t][0,1]:.2f} m along, box {7.24 - R[R[:,0] == first_t][0,1]:.2f} m ahead)" if first_t is not None else "     box never remembered")
        for t0_ in np.arange(R[0, 0], R[-1, 0], 2.0):
            w = R[(R[:, 0] >= t0_) & (R[:, 0] < t0_ + 2.0)]
            if len(w): print(f"     t {w[0,0]:5.1f} s  rover {w[0,1]:5.2f} m along  box cells remembered {int(w[:,2].min())}-{int(w[:,2].max())}")
        check("the box enters the memory before the rover is 4 m from it", first_t is not None and 7.24 - R[R[:, 0] == first_t][0, 1] > 4.0)
        check("... and stays remembered until the bumper reaches it", first_t is not None and len(held) and held[:, 2].min() > 0,
              f"min {int(held[:,2].min()) if len(held) else 0} cells between {first_t} and {t_reach:.1f} s" if first_t is not None else "")
        print(f"     cells remembered at the end of the replay: {len(grid)}")
        if start is not None:
            on_box = [k for k, _ in swept if (lambda f, l: BOX_F[0] < f < BOX_F[1] and -0.55 < l < 0.30)(*loc((k[0] + 0.5) * RES, (k[1] + 0.5) * RES))]
            print(f"     remembered cells the rover then drove over: {len(swept)} ({len(on_box)} of them the box)")
        TR = np.array(traj)
        moved = float(np.sum(np.hypot(np.diff(TR[:, 1]), np.diff(TR[:, 2])))) if len(TR) > 1 else 0.0
        def is_box(k):
            f, l = loc((k[0] + 0.5) * RES, (k[1] + 0.5) * RES)
            return BOX_F[0] < f < BOX_F[1] and -0.55 < l < 0.30
        if moved > 1.0:
            # phantom = remembered BEFORE the rover passed within 0.7 m of it (half the hull + 12 cm):
            # an obstacle in or beside its later path that was not the box
            bad = []
            for k, t_mark in ever.items():
                cx, cy = (k[0] + 0.5) * RES, (k[1] + 0.5) * RES
                after = TR[TR[:, 0] > t_mark]
                if len(after) and np.min(np.hypot(after[:, 1] - cx, after[:, 2] - cy)) < 0.7 and not is_box(k):
                    bad.append(k)
            print(f"     rover drove {moved:.1f} m; remembered cells it later passed within 0.7 m of, other than the box: {len(bad)}")
            for k in bad[:8]:
                f, l = loc((k[0] + 0.5) * RES, (k[1] + 0.5) * RES)
                print(f"       ({f:.2f} along, {l:+.2f}) marked at {ever[k] - t_start:.1f} s")
            check("no remembered cell in the rover's later path, other than the box", len(bad) == 0, f"{len(bad)} cells")
        else:
            # parked (walk-by): nothing may stay remembered in front of the rover where people walked
            px, py = TR[-1, 1], TR[-1, 2]; yaw_ = pose["yaw"]
            front = []
            for k in grid:
                dx, dy = (k[0] + 0.5) * RES - px, (k[1] + 0.5) * RES - py
                f, l = dx * math.cos(yaw_) + dy * math.sin(yaw_), -dx * math.sin(yaw_) + dy * math.cos(yaw_)
                if 1.0 < f < 8.0 and abs(l) < 2.6 and not is_box(k):
                    front.append((round(f, 2), round(l, 2), round(ever[k] - t_start, 1)))
            print(f"     parked; remembered cells in front of the rover (1-8 m, +-2.6 m) at the end: {len(front)} {sorted(front)[:8]}")
            check("parked: no remembered cell in front of the rover at the end", len(front) == 0, f"{len(front)} cells")

print(f"\n{sum(checks)}/{len(checks)} passed")
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if all(checks) else 1)
