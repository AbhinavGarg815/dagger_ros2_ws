#!/usr/bin/env python3
"""Replay a recorded LiDAR bag through Patchwork++ and measure it. NO ROVER.

    python3 tools/test_ground_seg.py [bag] [seconds]

ROS_DOMAIN_ID 42. Reports output rate, per-cloud latency (input arrival -> output
arrival), the node's CPU, and how the points split into ground / non-ground,
with non-ground heights relative to the ground plane (sensor at 1.12 m).
"""
import os
os.environ["ROS_DOMAIN_ID"] = "42"
import signal, statistics, subprocess, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

BAG = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1
                         else "/home/jetson/dagger_bags/dagger_drive_20260709_165625")
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 60
H = 1.12
SRC = ("export ROS_DOMAIN_ID=42 && source /opt/ros/humble/setup.bash && "
       "source /home/jetson/ros2_ws/install/setup.bash")

node = subprocess.Popen(["bash", "-lc", f"{SRC} && exec ros2 run patchworkpp patchworkpp_node "
    f"--ros-args -r pointcloud_topic:=/rslidar_points -p base_frame:=rslidar "
    f"-p sensor_height:={H} -p max_range:=30.0 -p min_range:=1.0"],
    stdout=open("/tmp/pwpp_node.log", "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)
rclpy.init(); n = Node("gseg_test")
t_in, t_out, ng_z, counts = {}, [], [], []


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def xyz(m):
    a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
    off = {f.name: f.offset for f in m.fields}
    g = lambda k: a[:, off[k]:off[k] + 4].copy().view(np.float32).ravel()
    return g("x"), g("y"), g("z")


def on_in(m):
    t_in[stamp(m)] = time.time()


def on_ng(m):
    now = time.time(); s = stamp(m)
    if s in t_in:
        t_out.append(now - t_in[s])
    if len(counts) % 10 == 0:
        x, y, z = xyz(m); r = np.hypot(x, y)
        ng_z.append((z + H, r))
    counts.append(m.width * m.height)


gcounts = []
n.create_subscription(PointCloud2, "/rslidar_points", on_in, qos_profile_sensor_data, raw=False)
n.create_subscription(PointCloud2, "/patchworkpp/nonground", on_ng, 10)
n.create_subscription(PointCloud2, "/patchworkpp/ground", lambda m: gcounts.append(m.width * m.height), 10)
time.sleep(5)
pid = int(subprocess.run(["pgrep", "-n", "-f", "patchworkpp_node --ros"], capture_output=True,
                         text=True).stdout.split()[0])


def cpu_ticks():
    f = open(f"/proc/{pid}/stat").read().split()
    return int(f[13]) + int(f[14])


play = subprocess.Popen(["bash", "-lc", f"{SRC} && exec ros2 bag play {BAG} --topics /rslidar_points"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
t0 = time.time(); c0 = cpu_ticks()
while time.time() - t0 < DUR and play.poll() is None:
    rclpy.spin_once(n, timeout_sec=0.01)
el = time.time() - t0; c1 = cpu_ticks()
for p in (play, node):
    try: os.killpg(os.getpgid(p.pid), signal.SIGINT)
    except ProcessLookupError: pass
time.sleep(2)
for p in (play, node):
    try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except ProcessLookupError: pass
n.destroy_node(); rclpy.shutdown()

hz = os.sysconf("SC_CLK_TCK")
print(f"\n{os.path.basename(BAG)}: {el:.0f} s replayed")
print(f"  output rate        {len(counts)/el:5.1f} Hz   (input is 10 Hz)")
if t_out:
    lt = sorted(t_out)
    print(f"  latency            median {statistics.median(lt)*1e3:5.1f} ms, p95 {lt[int(len(lt)*.95)]*1e3:5.1f} ms")
print(f"  node CPU           {100*(c1-c0)/hz/el:5.1f} % of ONE core")
if counts and gcounts:
    print(f"  points per cloud   non-ground {statistics.median(counts):.0f}, ground {statistics.median(gcounts):.0f}")
if ng_z:
    z = np.concatenate([a for a, r in ng_z]); r = np.concatenate([r for a, r in ng_z])
    print(f"  non-ground height above the ground plane, within 10 m:")
    zz = z[r < 10]
    for lo, hi in ((-9, 0.0), (0.0, 0.10), (0.10, 0.20), (0.20, 0.35), (0.35, 0.75), (0.75, 9)):
        k = ((zz >= lo) & (zz < hi)).sum()
        print(f"    {lo:+5.2f} .. {hi:+5.2f} m : {100*k/len(zz):5.1f} %")
