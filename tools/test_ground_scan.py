#!/usr/bin/env python3
"""Compare the costmap scan with and without ground segmentation. NO ROVER.

    python3 tools/test_ground_scan.py [bag] [seconds]

ROS_DOMAIN_ID 42. Replays raw clouds through Patchwork++ and three
pointcloud_to_laserscan instances configured like the costmap's:
  today   raw cloud,       floor 0.35 m  (the old band)
  naive   raw cloud,       floor 0.12 m  (just lowering the band)
  seg     non-ground cloud, floor 0.12 m  (ground_seg:=true)
A static TF puts the LiDAR at its measured 1.12 m. The lowest beam (-15 deg)
meets flat ground ~3.9-4.5 m out, so a ground ring shows up as returns there.
"""
import os
os.environ["ROS_DOMAIN_ID"] = "42"
import math, signal, subprocess, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

BAG = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1
                         else "/home/jetson/dagger_bags/dagger_drive_20260709_165625")
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 60
SRC = ("export ROS_DOMAIN_ID=42 && source /opt/ros/humble/setup.bash && "
       "source /home/jetson/ros2_ws/install/setup.bash")
P2L = ("-p target_frame:=base_footprint -p transform_tolerance:=0.1 -p max_height:=0.75 "
       "-p angle_min:=-3.14159 -p angle_max:=3.14159 -p angle_increment:=0.0087 "
       "-p scan_time:=0.1 -p range_min:=0.5 -p range_max:=30.0 -p use_inf:=true")
procs = []


def spawn(cmd, log):
    procs.append(subprocess.Popen(["bash", "-lc", f"{SRC} && exec {cmd}"], stdout=open(log, "w"),
                                  stderr=subprocess.STDOUT, preexec_fn=os.setsid))


spawn("ros2 run tf2_ros static_transform_publisher --x 0.2925 --z 1.12 "
      "--frame-id base_footprint --child-frame-id rslidar", "/tmp/gs_tf.log")
spawn("ros2 run patchworkpp patchworkpp_node --ros-args -r pointcloud_topic:=/rslidar_points "
      "-p base_frame:=rslidar -p sensor_height:=1.12 -p min_range:=1.0 -p max_range:=30.0",
      "/tmp/gs_pwpp.log")
for name, cloud, floor in (("today", "/rslidar_points", 0.35), ("naive", "/rslidar_points", 0.12),
                           ("seg", "/patchworkpp/nonground", 0.12)):
    spawn(f"ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args "
          f"-r __node:=p2l_{name} -r cloud_in:={cloud} -r scan:=/test/scan_{name} "
          f"-p min_height:={floor} {P2L}", f"/tmp/gs_p2l_{name}.log")

rclpy.init(); n = Node("gscan_test")
S = {k: [] for k in ("today", "naive", "seg")}
for k in S:
    n.create_subscription(LaserScan, f"/test/scan_{k}",
                          lambda m, k=k: S[k].append((m.header.stamp.sec * 1000 + m.header.stamp.nanosec // 1_000_000,
                                                      np.asarray(m.ranges, dtype=np.float32))),
                          qos_profile_sensor_data)
time.sleep(6)
spawn(f"ros2 bag play {BAG} --topics /rslidar_points", "/tmp/gs_play.log")
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.01)
for p in procs:
    try: os.killpg(os.getpgid(p.pid), signal.SIGINT)
    except ProcessLookupError: pass
time.sleep(2)
for p in procs:
    try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except ProcessLookupError: pass
n.destroy_node(); rclpy.shutdown()


# Pair scans by the SOURCE CLOUD's timestamp. An earlier version paired them by
# index across independent streams (549 vs 577 scans), so it compared different
# instants and the numbers moved between identical runs.
common = set.intersection(*[{st for st, _ in v} for v in S.values() if v]) if all(S.values()) else set()
for k in S:
    S[k] = [r for st, r in sorted(S[k], key=lambda x: x[0]) if st in common]


def stats(scans):
    a = np.stack(scans)
    fin = np.isfinite(a)
    ring = fin & (a >= 3.6) & (a <= 4.6)
    near = fin & (a <= 8.0)
    return ring.mean() * 100, near.sum(1).mean(), a


print(f"\n{os.path.basename(BAG)}, {DUR:.0f} s, {len(common)} scans matched by cloud timestamp")
print(f"  {'config':8s} {'rays with a return in the 3.6-4.6 m ground-ring zone':>52s} {'returns <8 m / scan':>20s}")
res = {}
for k in ("today", "naive", "seg"):
    if not S[k]:
        print(f"  {k:8s} NO SCANS"); continue
    ring, near, a = stats(S[k]); res[k] = (ring, near, a)
    print(f"  {k:8s} {ring:51.1f} % {near:19.0f}")
RES_MISS = 999
if all(k in res for k in ("today", "seg")):
    t, s_ = res["today"][2], res["seg"][2]
    extra = (np.isfinite(s_) & (s_ <= 8)) & ~(np.isfinite(t) & (t <= 8))
    print(f"\n  obstacle returns under 8 m that ONLY ground seg sees: {extra.sum(1).mean():.0f} rays/scan "
          f"(the 12-35 cm layer)")
    # The safety question: does segmentation HIDE anything the old band sees?
    miss = (np.isfinite(t) & (t <= 8)) & ~(np.isfinite(s_) & (s_ <= t + 0.3))
    print(f"  obstacle returns under 8 m that the OLD band sees and ground seg LOSES: "
          f"{miss.sum(1).mean():.1f} rays/scan")
    ang = -3.14159 + np.arange(t.shape[1]) * 0.0087
    idx = np.nonzero(miss)
    if idx[0].size:
        rr = t[idx]
        print(f"    their ranges: median {np.median(rr):.2f} m, "
              f"<1.5 m: {100*(rr < 1.5).mean():.0f} %, 1.5-4 m: {100*((rr >= 1.5) & (rr < 4)).mean():.0f} %, "
              f">=4 m: {100*(rr >= 4).mean():.0f} %")
        print(f"    bearings: {100*(np.abs(ang[idx[1]]) > 1.571).mean():.0f} % behind the rover")
    RES_MISS = miss.sum(1).mean()
ok = "seg" in res and RES_MISS < 1.0
print(f"\n  [{'PASS' if ok else 'FAIL'}] ground seg hides (almost) nothing the old band sees")
sys.exit(0 if ok else 1)
