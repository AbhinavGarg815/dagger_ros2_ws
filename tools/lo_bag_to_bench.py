#!/usr/bin/env python3
"""(Q73) A recorded drive with lidar_odom running (T2 lidar_odom:=shadow|true, recorded with
tools/detour_record.sh) -> tools/lo_eval.py's format, with two "methods" scored against the same
direct-registration references: the LIVE /lidar_odom/odometry and /odometry/filtered.

    python tools/lo_bag_to_bench.py <bag> <out.npz>
    python tools/lo_eval.py <out.npz>

Also prints lidar_odom's status over the drive (accepted, rejected, resets, degenerate, gaps, CPU)
and the start-to-end check: the last scan registered directly against the first one (a drive that
ends on its start mark), compared with what each odometry says. Needs the small_gicp pip wheel.
"""
import sys, math, numpy as np, rosbag2_py, small_gicp
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from dagger_msgs.msg import LidarOdomStatus

bag, out = sys.argv[1], sys.argv[2]
YAW, T = 1.5708, np.array([0.2925, 0.0, 1.12])
RZ = np.array([[math.cos(YAW), -math.sin(YAW), 0], [math.sin(YAW), math.cos(YAW), 0], [0, 0, 1]])
DT = np.dtype({"names": ["x", "y", "z"], "formats": [np.float32] * 3, "offsets": [0, 4, 8], "itemsize": 26})


def mat(o):
    q, p = o.pose.pose.orientation, o.pose.pose.position
    R = np.array([[1 - 2 * (q.y**2 + q.z**2), 2 * (q.x*q.y - q.z*q.w), 2 * (q.x*q.z + q.y*q.w)],
                  [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y*q.z - q.x*q.w)],
                  [2 * (q.x*q.z - q.y*q.w), 2 * (q.y*q.z + q.x*q.w), 1 - 2 * (q.x**2 + q.y**2)]])
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = (p.x, p.y, p.z); return M


rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
t0 = rd.get_metadata().starting_time.nanoseconds * 1e-9
rd.set_filter(rosbag2_py.StorageFilter(topics=["/rslidar_points", "/odometry/filtered", "/lidar_odom/odometry",
                                               "/lidar_odom/status"]))
clouds, lo, fo, st, keep = [], {}, [], [], {}
first_last = [None, None]
while rd.has_next():
    tp, d, ts = rd.read_next(); tb = ts * 1e-9 - t0
    if tp == "/lidar_odom/odometry":
        m = deserialize_message(d, Odometry); lo[(m.header.stamp.sec, m.header.stamp.nanosec)] = mat(m)
    elif tp == "/odometry/filtered":
        m = deserialize_message(d, Odometry); M = mat(m); q = m.pose.pose.orientation
        fo.append((tb, M, m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z),
                   m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif tp == "/lidar_odom/status":
        st.append((tb, deserialize_message(d, LidarOdomStatus)))
    else:
        m = deserialize_message(d, PointCloud2); k = len(clouds)
        clouds.append((tb, (m.header.stamp.sec, m.header.stamp.nanosec)))
        if True:                                  # every scan: the last one is the start->end check
            a = np.frombuffer(bytes(m.data), dtype=DT); P = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)
            P = P[np.isfinite(P).all(1)]; B = P @ RZ.T + T; r = np.linalg.norm(B, axis=1)
            B = B[~((B[:, 0] > -0.9) & (B[:, 0] < 0.5) & (np.abs(B[:, 1]) < 0.6)) & (r >= 0.3) & (r <= 60)]
            V = small_gicp.voxelgrid_sampling(B, 0.1).points()[:, :3].astype(np.float32)
            if k % 5 == 0: keep[k] = V
            if first_last[0] is None: first_last[0] = (k, V)
            first_last[1] = (k, V)

fot = np.array([f[0] for f in fo])
stamps, P_lo, P_fo, last = [], [], [], np.eye(4)
for tb, key in clouds:
    if key in lo: last = lo[key]
    stamps.append(tb); P_lo.append(last.copy())
    P_fo.append(fo[int(np.argmin(np.abs(fot - tb)))][1])
odom = np.array([(f[0], f[2], f[3], f[4], f[5], f[6]) for f in fo])
np.savez(out, stamps=np.array(stamps), odom=odom, methods=np.array(["lidar_odom", "odom_filtered"]),
         keep_idx=np.array(sorted(keep)), keep=np.array([keep[i] for i in sorted(keep)], dtype=object),
         pose_lidar_odom=np.array(P_lo), pose_odom_filtered=np.array(P_fo),
         cpu_lidar_odom=np.array([s.cpu_ms for _, s in st]), cpu_odom_filtered=np.zeros(1))

S = [s for _, s in st]
acc = np.array([s.accepted for s in S]); dts = np.array([s.dt for s in S if s.accepted and s.dt > 0])
inf = np.array([s.info_min for s in S if s.accepted and s.info_min > 0]); ms = np.array([s.cpu_ms for s in S])
print(f"{bag.rstrip('/').split('/')[-1]}: {len(clouds)} clouds, {stamps[-1] - stamps[0]:.0f} s; lidar_odom: {len(lo)} odometry msgs, "
      f"accepted {acc.mean():.2%}, rejected {S[-1].rejected_total - S[0].rejected_total}, resets {S[-1].resets_total - S[0].resets_total}, "
      f"degenerate {sum(s.degenerate for s in S)}, gaps > 0.15 s {np.sum(dts > 0.15)}, cpu {ms.mean():.0f}/{np.percentile(ms, 95):.0f} ms")
print(f"  translational information: min {inf.min():.0f}, p1 {np.percentile(inf, 1):.0f}, p5 {np.percentile(inf, 5):.0f}, median {np.median(inf):.0f}")
deg = [(tb, s) for tb, s in st if s.degenerate]
if deg:
    print(f"  degenerate between {deg[0][0]:.1f} and {deg[-1][0]:.1f} s ({len(deg)} scans); weak axis e.g. {np.round(deg[len(deg)//2][1].weak_axis, 2)}")
(ka, A), (kb, Bv) = first_last
for name, P in (("lidar_odom", P_lo), ("odom_filtered", P_fo)):
    rel = np.linalg.inv(P[ka]) @ P[kb]
    r = small_gicp.align(A.astype(np.float64), Bv.astype(np.float64), rel, registration_type="GICP", downsampling_resolution=0.1,
                         max_correspondence_distance=1.0, max_iterations=50, num_threads=4)
    frac = r.num_inliers / len(Bv); E = np.linalg.inv(r.T_target_source) @ rel
    print(f"  start->end via {name:13s}: says {np.linalg.norm(rel[:2, 3]):.2f} m apart; direct registration from that guess: "
          f"{np.linalg.norm(r.T_target_source[:2, 3]):.2f} m apart ({frac:.0%} inliers) -> error {np.linalg.norm(E[:2, 3]):.2f} m, "
          f"{abs(math.degrees(math.atan2(E[1, 0], E[0, 0]))):.1f} deg")
