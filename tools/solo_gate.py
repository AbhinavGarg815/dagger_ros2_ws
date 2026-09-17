#!/usr/bin/env python3
"""Safety gate for solo tests (tools/solo_goal.sh). NON-MOTION: only reads topics.

    python3 tools/solo_gate.py --save    # remember the start pose (run at the start position)
    python3 tools/solo_gate.py           # PASS (exit 0) only if ALL hold:
        - FCU armed and in GUIDED
        - rover within 0.5 m / 8 deg of the saved start pose
        - /scan_safety fresher than 0.5 s and <= 3 points in the zone 0.85-2.6 m ahead, +-0.85 m
"""
import json, math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

START = os.path.expanduser("~/dagger_bags/.solo_start.json")


def main():
    rclpy.init()
    n = Node("solo_gate")
    s = {}
    n.create_subscription(Odometry, "/odometry/filtered", lambda m: s.__setitem__("od", m), 10)
    n.create_subscription(State, "/mavros/state", lambda m: s.__setitem__("st", m), qos_profile_sensor_data)
    n.create_subscription(LaserScan, "/scan_safety", lambda m: s.__setitem__("sc", (time.time(), m)),
                          qos_profile_sensor_data)
    end = time.time() + 3.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    od = s.get("od")
    if od is None:
        print("GATE FAIL: no /odometry/filtered")
        sys.exit(1)
    p, q = od.pose.pose.position, od.pose.pose.orientation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    if "--save" in sys.argv:
        with open(START, "w") as f:
            json.dump({"x": p.x, "y": p.y, "yaw": yaw}, f)
        print(f"start pose saved: ({p.x:.2f}, {p.y:.2f}) yaw {yaw:.1f} deg")
        return
    fail = []
    st = s.get("st")
    if not st or not st.armed or st.mode != "GUIDED":
        fail.append(f"not armed+GUIDED (mode={st.mode if st else '?'}, armed={st.armed if st else '?'})")
    with open(START) as f:
        s0 = json.load(f)
    dp = math.hypot(p.x - s0["x"], p.y - s0["y"])
    dyaw = abs((yaw - s0["yaw"] + 180) % 360 - 180)
    if dp > 0.5 or dyaw > 8:
        fail.append(f"moved {dp:.2f} m / {dyaw:.1f} deg from start")
    sc = s.get("sc")
    if not sc or time.time() - sc[0] > 0.5:
        fail.append("/scan_safety stale")
    else:
        m = sc[1]
        k = 0
        for i, r in enumerate(m.ranges):
            if math.isfinite(r) and m.range_min <= r <= m.range_max:
                a = m.angle_min + i * m.angle_increment
                if 0.85 <= r * math.cos(a) <= 2.6 and abs(r * math.sin(a)) <= 0.85:
                    k += 1
        if k > 3:
            fail.append(f"{k} points ahead in the slow/stop zone")
    print(time.strftime("%H:%M:%S"), "GATE", "PASS" if not fail else "FAIL: " + "; ".join(fail))
    sys.exit(0 if not fail else 1)


if __name__ == "__main__":
    main()
