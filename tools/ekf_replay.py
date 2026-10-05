#!/usr/bin/env python3
"""(Q77) Replay ekf_local (robot_localization, fused odometry) offline from a recorded drive with
different input set-ups, to tune the odometry heading without the rover. NO ROVER.

    python3 tools/ekf_replay.py <bag> <out_dir> [--variants V0,V1,...] [--rate 2]

The bag must hold the EKF inputs (tools/detour_record.sh since 2026-10-05): /fcu/odom,
/mavros/imu/data_raw, /gps/heading_imu, /lidar_odom/odometry, /tf_static. Every variant runs its
own ekf_node (installed ekf_fused.yaml + ekf_fused_lidar.yaml + the variant's overrides) in its own
ROS domain on sim time, fed by `ros2 bag play --clock`; its /odometry/filtered is saved as
<out_dir>/<variant>.npz (t since bag start, x, y, yaw). Score with tools/ekf_replay_score.py.

Variants (heading inputs of the odometry EKF):
    V0  as live: gyro rate + dual-antenna GNSS heading (sigma 0.5 deg) + LiDAR vx, vy
    V1  V0 + the LiDAR yaw rate
    V2  V1 with the GNSS heading at sigma 3 deg (its covariance x36, by a relay)
    V3  no GNSS heading: gyro + LiDAR yaw rate (absolute heading left to map->odom)
    V4  no GNSS heading: gyro only
    V5  LiDAR POSE (x, y, yaw; odom1_relative) + FCU velocity + gyro, no GNSS heading
    V6  V5 + the GNSS heading as live
"""
import argparse, os, subprocess, sys, time

VARIANTS = {
    "V0": {"overrides": {}, "heading_scale": None},
    "V1": {"overrides": {"odom1_config": "lidar_vyaw"}, "heading_scale": None},
    "V2": {"overrides": {"odom1_config": "lidar_vyaw"}, "heading_scale": 36.0},
    "V3": {"overrides": {"odom1_config": "lidar_vyaw", "imu1": "/gps/heading_imu_disabled"}, "heading_scale": None},
    "V4": {"overrides": {"imu1": "/gps/heading_imu_disabled"}, "heading_scale": None},
    "V5": {"overrides": {"odom1_config": "lidar_pose", "odom1_relative": "true", "imu1": "/gps/heading_imu_disabled"},
           "heading_scale": None},
    "V6": {"overrides": {"odom1_config": "lidar_pose", "odom1_relative": "true"}, "heading_scale": None},
}
ODOM1_POSE = ("[true, true, false, false, false, true, false, false, false, "
              "false, false, false, false, false, false]")
ODOM1_VYAW = ("[false, false, false, false, false, false, true, true, false, "
              "false, false, true, false, false, false]")
PARAMS = "/home/jetson/ros2_ws/install/dagger_bringup/share/dagger_bringup/params"
SRC = "source /opt/ros/humble/setup.bash && source /home/jetson/ros2_ws/install/setup.bash"


def run_one(bag, out_dir, name, domain, rate):
    """Child process: one variant in its own domain."""
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    import signal
    import numpy as np, rclpy, rosbag2_py
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    v = VARIANTS[name]
    ov = os.path.join(out_dir, f"{name}_params.yaml")
    with open(ov, "w") as f:
        f.write("ekf_local:\n  ros__parameters:\n    use_sim_time: true\n    publish_tf: false\n")
        for k, val in v["overrides"].items():
            f.write(f"    {k}: {ODOM1_VYAW if val == 'lidar_vyaw' else ODOM1_POSE if val == 'lidar_pose' else val}\n")
    env = f"export ROS_DOMAIN_ID={domain} && {SRC}"
    procs = [subprocess.Popen(
        ["bash", "-lc", f"{env} && exec ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_local "
         f"--params-file {PARAMS}/ekf_fused.yaml --params-file {PARAMS}/ekf_fused_lidar.yaml --params-file {ov} "
         "-r odometry/filtered:=/replay/odom"],
        stdout=open(os.path.join(out_dir, f"{name}_ekf.log"), "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)]

    def stop():
        for sig in (signal.SIGINT, signal.SIGKILL):
            for p in procs:
                try: os.killpg(os.getpgid(p.pid), sig)
                except ProcessLookupError: pass
            time.sleep(1.0)
    import atexit; atexit.register(stop)

    rclpy.init()
    n = Node(f"ekf_replay_{name}", parameter_overrides=[Parameter("use_sim_time", value=True)])
    rows = []
    n.create_subscription(Odometry, "/replay/odom", lambda m: rows.append((
        m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m.pose.pose.position.x, m.pose.pose.position.y,
        np.arctan2(2 * (m.pose.pose.orientation.w * m.pose.pose.orientation.z), 1 - 2 * m.pose.pose.orientation.z ** 2))), 100)
    if v["heading_scale"]:
        pub = n.create_publisher(Imu, "/gps/heading_imu", 10)
        def relay(m, k=v["heading_scale"]):
            c = list(m.orientation_covariance); c[8] *= k; m.orientation_covariance = c; pub.publish(m)
        n.create_subscription(Imu, "/gps/heading_imu_raw", relay, 10)
    time.sleep(4.0)
    remap = ["--remap", "/gps/heading_imu:=/gps/heading_imu_raw"] if v["heading_scale"] else []
    play = subprocess.Popen(
        ["bash", "-lc", f"{env} && exec ros2 bag play {bag} --clock 100 --rate {rate} --topics /fcu/odom "
         f"/mavros/imu/data_raw /gps/heading_imu /lidar_odom/odometry /tf_static {' '.join(remap)}"],
        stdout=open(os.path.join(out_dir, f"{name}_play.log"), "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    procs.append(play)
    while play.poll() is None:
        rclpy.spin_once(n, timeout_sec=0.05)
    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    t0 = rd.get_metadata().starting_time.nanoseconds * 1e-9
    R = np.array(rows); R[:, 0] -= t0
    np.savez(os.path.join(out_dir, f"{name}.npz"), odom=R)
    print(f"{name}: {len(R)} poses, {R[0, 0]:.1f}-{R[-1, 0]:.1f} s")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("bag"); ap.add_argument("out_dir")
    ap.add_argument("--variants", default=",".join(VARIANTS)); ap.add_argument("--rate", default="2")
    ap.add_argument("--one", default=None); ap.add_argument("--domain", type=int, default=90)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    if a.one:
        run_one(a.bag, a.out_dir, a.one, a.domain, a.rate); sys.exit(0)
    kids = [subprocess.Popen([sys.executable, __file__, a.bag, a.out_dir, "--one", v, "--domain", str(90 + i), "--rate", a.rate])
            for i, v in enumerate(a.variants.split(","))]
    sys.exit(max(k.wait() for k in kids))
