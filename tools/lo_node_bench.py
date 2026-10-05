#!/usr/bin/env python3
"""(Q73) Replay a bag through the C++ lidar_odom node, beside the Python small_gicp reference
(VGICP 0.5 m, the benchmarked "vgicp05"), and save both in tools/lo_bench.py's format so
tools/lo_eval.py scores them against the same direct-registration references. NO ROVER.

    python tools/lo_node_bench.py <bag> <out.npz> [--domain 86]
    python tools/lo_eval.py <out.npz>

Lockstep: one cloud at a time, the next only after the node's status for it. The node gets the
bag's own /tf_static; the reference gets the same points (URDF mount, body box, 0.3-60 m).
Needs the small_gicp pip wheel (venv, see COMMANDS.md section 11) and OMP_NUM_THREADS=1.
Also saves per scan the node's status: accepted, info_min, degenerate, cpu_ms.
"""
import argparse, os, sys
ap = argparse.ArgumentParser(); ap.add_argument("bag"); ap.add_argument("out")
ap.add_argument("--domain", default="86")
a = ap.parse_args()
os.environ["ROS_DOMAIN_ID"] = a.domain
import atexit, math, signal, subprocess, time
import numpy as np, rclpy, rosbag2_py, small_gicp
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from dagger_msgs.msg import LidarOdomStatus

SRC = (f"export ROS_DOMAIN_ID={a.domain} && source /opt/ros/humble/setup.bash && "
       "source /home/jetson/ros2_ws/install/setup.bash")
proc = subprocess.Popen(
    ["bash", "-lc", f"{SRC} && exec ros2 run dagger_perception lidar_odom --ros-args "
     "-r cloud_in:=/t/cloud -r odometry:=/t/odom -r status:=/t/status"],
    stdout=open(f"/tmp/lo_node_bench_{a.domain}.log", "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)


@atexit.register
def stop():
    for sig in (signal.SIGINT, signal.SIGKILL):
        try: os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError: pass
        time.sleep(1.0)


rclpy.init(); n = Node("lo_node_bench")
key = lambda h: (h.stamp.sec, h.stamp.nanosec)
stats, odoms = {}, {}
n.create_subscription(LidarOdomStatus, "/t/status", lambda m: stats.__setitem__(key(m.header), m), 100)
n.create_subscription(Odometry, "/t/odom", lambda m: odoms.__setitem__(key(m.header), m), 100)
pub = n.create_publisher(PointCloud2, "/t/cloud", QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1))
tfp = n.create_publisher(TFMessage, "/tf_static", QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1))


def reader(topics):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    rd.set_filter(rosbag2_py.StorageFilter(topics=topics)); return rd


rd = reader(["/tf_static"]); allt = TFMessage()
while rd.has_next():
    allt.transforms.extend(deserialize_message(rd.read_next()[1], TFMessage).transforms)
tfp.publish(allt)
time.sleep(3.0)

YAW, T = 1.5708, np.array([0.2925, 0.0, 1.12])
RZ = np.array([[math.cos(YAW), -math.sin(YAW), 0], [math.sin(YAW), math.cos(YAW), 0], [0, 0, 1]])
DT = np.dtype({"names": ["x", "y", "z"], "formats": [np.float32] * 3, "offsets": [0, 4, 8], "itemsize": 26})
vm = small_gicp.GaussianVoxelMap(0.5); vm.set_lru(horizon=100, clear_cycle=10)
Tp, dp = np.eye(4), np.eye(4)
rd = reader(["/rslidar_points", "/odometry/filtered"])
t0 = rd.get_metadata().starting_time.nanoseconds * 1e-9
stamps, P_node, P_ref, cpu_node, cpu_ref, odom, keep = [], [], [], [], [], [], {}
acc, info, deg = [], [], []
last_node = np.eye(4); k = 0
while rd.has_next():
    tp, d, ts = rd.read_next(); tb = ts * 1e-9 - t0
    if tp == "/odometry/filtered":
        m = deserialize_message(d, Odometry); q = m.pose.pose.orientation
        odom.append((tb, m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z),
                     m.twist.twist.linear.x, m.twist.twist.angular.z)); continue
    m = deserialize_message(d, PointCloud2); kk = key(m.header)
    pub.publish(m)
    a_ = np.frombuffer(bytes(m.data), dtype=DT); P = np.stack([a_["x"], a_["y"], a_["z"]], 1).astype(np.float64)
    P = P[np.isfinite(P).all(1)]; B = P @ RZ.T + T; r = np.linalg.norm(B, axis=1)
    B = B[~((B[:, 0] > -0.9) & (B[:, 0] < 0.5) & (np.abs(B[:, 1]) < 0.6)) & (r >= 0.3) & (r <= 60)]
    c0 = time.process_time()
    src, _ = small_gicp.preprocess_points(B, downsampling_resolution=0.2, num_neighbors=10, num_threads=1)
    if vm.size() == 0:
        vm.insert(src)
    else:
        res = small_gicp.align(vm, src, Tp @ dp, max_correspondence_distance=1.0, num_threads=1)
        dp = np.linalg.inv(Tp) @ res.T_target_source; Tp = res.T_target_source; vm.insert(src, Tp)
    cpu_ref.append((time.process_time() - c0) * 1000)
    end = time.time() + 5.0
    while time.time() < end and kk not in stats:
        rclpy.spin_once(n, timeout_sec=0.005)
    end = time.time() + 0.05
    while time.time() < end and kk not in odoms:
        rclpy.spin_once(n, timeout_sec=0.005)
    s, o = stats.pop(kk, None), odoms.pop(kk, None)
    if o is not None:
        q, p = o.pose.pose.orientation, o.pose.pose.position
        R = np.array([[1 - 2 * (q.y**2 + q.z**2), 2 * (q.x*q.y - q.z*q.w), 2 * (q.x*q.z + q.y*q.w)],
                      [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y*q.z - q.x*q.w)],
                      [2 * (q.x*q.z - q.y*q.w), 2 * (q.y*q.z + q.x*q.w), 1 - 2 * (q.x**2 + q.y**2)]])
        last_node = np.eye(4); last_node[:3, :3] = R; last_node[:3, 3] = (p.x, p.y, p.z)
    stamps.append(tb); P_node.append(last_node.copy()); P_ref.append(np.array(Tp))
    cpu_node.append(s.cpu_ms if s else np.nan)
    acc.append(bool(s and s.accepted)); info.append(s.info_min if s else np.nan); deg.append(bool(s and s.degenerate))
    if k % 5 == 0:
        keep[k] = small_gicp.voxelgrid_sampling(B, 0.1).points()[:, :3].astype(np.float32)
    k += 1
    if k % 500 == 0:
        print(f"  {k} scans, node {np.nanmean(cpu_node[-500:]):.1f} ms, accepted {np.mean(acc[-500:]):.1%}", flush=True)

np.savez(a.out, stamps=np.array(stamps), odom=np.array(odom), methods=np.array(["node", "vgicp05"]),
         keep_idx=np.array(sorted(keep)), keep=np.array([keep[i] for i in sorted(keep)], dtype=object),
         pose_node=np.array(P_node), pose_vgicp05=np.array(P_ref), cpu_node=np.array(cpu_node), cpu_vgicp05=np.array(cpu_ref),
         node_accepted=np.array(acc), node_info=np.array(info), node_degenerate=np.array(deg))
inf = np.array(info)[np.array(acc)]
print(f"{os.path.basename(a.bag)}: {k} scans, node accepted {np.mean(acc):.2%} ({k - sum(acc)} rejected), degenerate {sum(deg)}, "
      f"info_min min {np.nanmin(inf):.0f} p1 {np.nanpercentile(inf, 1):.0f} median {np.nanmedian(inf):.0f}; "
      f"CPU node {np.nanmean(cpu_node):.1f}/{np.nanpercentile(cpu_node, 95):.1f} ms mean/p95")
