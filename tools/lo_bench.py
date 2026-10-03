"""(Q73) Offline LiDAR-odometry benchmark on one Dagger bag.
   python lo_bench.py <bag> <out.npz> [methods,comma,separated]
Every method gets the same input: /rslidar_points in base_footprint (URDF mount), body box
removed, 0.3-60 m, per-point time normalized to [0, 1] within the scan. Single thread each.
Saves per method: scan stamps, 4x4 poses (base_footprint in the method's world), CPU ms per
scan; plus every 5th scan voxelized at 0.1 m (for the pairwise reference) and the bag's odom."""
import sys, time, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import small_gicp

GRAV = {"roll": 0.0, "pitch": 0.0, "t": None, "init": False}     # complementary filter on /mavros/imu/data_raw


def imu_update(m, t):
    ax, ay, az = m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z
    r_acc, p_acc = math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))
    if not GRAV["init"]:
        GRAV.update(roll=r_acc, pitch=p_acc, t=t, init=True); return
    dt = min(max(t - GRAV["t"], 0.0), 0.5); GRAV["t"] = t
    k = dt / (2.0 + dt)                                            # tau 2 s on the accelerometer
    GRAV["roll"] = (1 - k) * (GRAV["roll"] + m.angular_velocity.x * dt) + k * r_acc
    GRAV["pitch"] = (1 - k) * (GRAV["pitch"] + m.angular_velocity.y * dt) + k * p_acc


def gravity_align(T):
    """Keep yaw and translation, take roll/pitch from the IMU filter."""
    yaw = math.atan2(T[1, 0], T[0, 0]); r, p = GRAV["roll"], GRAV["pitch"]
    cz, sz, cy, sy, cx, sx = math.cos(yaw), math.sin(yaw), math.cos(p), math.sin(p), math.cos(r), math.sin(r)
    R = np.array([[cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                  [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                  [-sy, cy * sx, cy * cx]])
    out = T.copy(); out[:3, :3] = R; return out

bag, out = sys.argv[1], sys.argv[2]
METHODS = sys.argv[3].split(",") if len(sys.argv) > 3 else ["kiss", "kiss_july", "genz", "vgicp"]
YAW, T = 1.5708, np.array([0.2925, 0.0, 1.12])
RZ = np.array([[math.cos(YAW), -math.sin(YAW), 0], [math.sin(YAW), math.cos(YAW), 0], [0, 0, 1]])
DT = np.dtype({"names": ["x", "y", "z", "t"], "formats": [np.float32] * 3 + [np.float64],
               "offsets": [0, 4, 8, 18], "itemsize": 26})


def make(name):
    if name in ("kiss", "kiss1", "kiss_july"):
        from kiss_icp.config import KISSConfig
        from kiss_icp.kiss_icp import KissICP
        c = KISSConfig()
        c.registration.max_num_threads = 1
        if name in ("kiss", "kiss1"):
            c.data.max_range, c.data.min_range, c.data.deskew = 60.0, 0.0, True
            c.mapping.voxel_size = 0.5 if name == "kiss" else 1.0
        else:                                   # the July setup: raw cloud, min_range 1.0, voxel = 100/100
            c.data.max_range, c.data.min_range, c.data.deskew = 100.0, 1.0, True
            c.mapping.voxel_size = 1.0
        o = KissICP(c)
        return lambda P, t: (o.register_frame(P, t), o.last_pose)[1]
    if name.startswith("genz"):
        from genz_icp.config import GenZConfig
        from genz_icp.genz_icp import GenZICP
        c = GenZConfig()
        c.data.max_range, c.data.min_range = 60.0, 0.3
        c.data.deskew = name == "genz_deskew"
        o = GenZICP(c)
        return lambda P, t: (o.register_frame(P, t), o.last_pose)[1]
    if name.startswith("vgicp"):
        res = 0.5 if name.startswith("vgicp05") else 1.0; ds = 0.2 if res == 0.5 else 0.25
        grav = name.endswith("_grav")
        st = {"T": np.eye(4), "d": np.eye(4), "map": small_gicp.GaussianVoxelMap(res), "H": []}
        st["map"].set_lru(horizon=100, clear_cycle=10)
        def step(P, t):
            src, _ = small_gicp.preprocess_points(P, downsampling_resolution=ds, num_neighbors=10, num_threads=1)
            if st["map"].size() == 0:
                st["map"].insert(src); st["H"].append(np.nan); return st["T"]
            r = small_gicp.align(st["map"], src, st["T"] @ st["d"], max_correspondence_distance=1.0, num_threads=1)
            Tn = gravity_align(r.T_target_source) if grav else r.T_target_source
            st["d"] = np.linalg.inv(st["T"]) @ Tn; st["T"] = Tn
            st["map"].insert(src, st["T"])
            st["H"].append(float(np.linalg.eigvalsh(r.H)[0]))
            return st["T"]
        step.state = st
        return step
    raise ValueError(name)


meth = {m: make(m) for m in METHODS}
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
t0 = rd.get_metadata().starting_time.nanoseconds * 1e-9
rd.set_filter(rosbag2_py.StorageFilter(topics=["/rslidar_points", "/odometry/filtered", "/mavros/imu/data_raw"]))
stamps, poses, cpu, keep, odom = [], {m: [] for m in METHODS}, {m: [] for m in METHODS}, {}, []
n = 0
while rd.has_next():
    tp, d, ts = rd.read_next(); tb = ts * 1e-9 - t0
    if tp == "/mavros/imu/data_raw":
        imu_update(deserialize_message(d, Imu), tb); continue
    if tp == "/odometry/filtered":
        m = deserialize_message(d, Odometry); q = m.pose.pose.orientation
        odom.append((tb, m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z), m.twist.twist.linear.x, m.twist.twist.angular.z))
        continue
    a = np.frombuffer(bytes(deserialize_message(d, PointCloud2).data), dtype=DT)
    P = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64); tt = a["t"]
    ok = np.isfinite(P).all(1) & np.isfinite(tt); P, tt = P[ok], tt[ok]
    B = P @ RZ.T + T
    r = np.linalg.norm(B, axis=1)
    raw_ok = (r <= 100.0)                                       # kiss_july: raw cloud, its own min_range
    body = (B[:, 0] > -0.90) & (B[:, 0] < 0.50) & (np.abs(B[:, 1]) < 0.60)
    good = ~body & (r >= 0.3) & (r <= 60.0)
    tn = (tt - tt.min()) / max(tt.max() - tt.min(), 1e-6)
    stamps.append(tb)
    for name, f in meth.items():
        Pin, tin = (B[raw_ok], tn[raw_ok]) if name == "kiss_july" else (B[good], tn[good])
        c0 = time.process_time()
        Tm = f(np.ascontiguousarray(Pin), np.ascontiguousarray(tin))
        cpu[name].append((time.process_time() - c0) * 1000)
        poses[name].append(np.array(Tm, dtype=np.float64))
    if n % 5 == 0:
        keep[n] = small_gicp.voxelgrid_sampling(B[good], 0.1).points()[:, :3].astype(np.float32)
    n += 1
    if n % 500 == 0:
        print(f"  {n} scans, cpu ms/scan: " + ", ".join(f"{m} {np.mean(cpu[m][-500:]):.1f}" for m in METHODS), flush=True)

save = {"stamps": np.array(stamps), "odom": np.array(odom), "methods": np.array(METHODS),
        "keep_idx": np.array(sorted(keep)), "keep": np.array([keep[k] for k in sorted(keep)], dtype=object)}
for m in METHODS:
    save[f"pose_{m}"] = np.array(poses[m]); save[f"cpu_{m}"] = np.array(cpu[m])
for m in METHODS:
    if m.startswith("vgicp"):
        save[f"{m}_Hmin"] = np.array(meth[m].state["H"])
np.savez(out, **save)
print(f"{bag}: {n} scans; CPU ms/scan (mean / p95): " +
      ", ".join(f"{m} {np.mean(cpu[m]):.1f}/{np.percentile(cpu[m], 95):.1f}" for m in METHODS))
