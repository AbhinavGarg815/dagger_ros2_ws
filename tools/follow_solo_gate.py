#!/usr/bin/env python3
"""Safety gate for the solo follow-me test (tools/follow_solo.sh). NON-MOTION: reads only.

    python3 tools/follow_solo_gate.py     # PASS (exit 0) only if ALL hold:
        - FCU armed and in GUIDED
        - a FRESH target on /follow/target
        - that target is inside the controller's distance deadband, so enabling
          follow cannot make the rover lurch: it will command zero at the moment
          it is switched on, and only start moving once the person walks away
        - the target is roughly ahead, so it will not pivot the instant it starts
        - /scan_safety fresh and the stop zone clear

The distance check is the important one. Enabling follow while the person is at
6 m would command 0.4 m/s immediately, with nobody near the laptop.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State

# Controller defaults: follow_distance 3.0, distance_deadband 0.35.
MIN_RANGE, MAX_RANGE = 2.70, 3.30
MAX_BEARING_DEG = 10.0


def main():
    rclpy.init()
    n = Node("follow_solo_gate")
    s = {}
    n.create_subscription(State, "/mavros/state",
                          lambda m: s.__setitem__("st", m), qos_profile_sensor_data)
    n.create_subscription(PoseStamped, "/follow/target",
                          lambda m: s.__setitem__("tg", (time.time(), m)), qos_profile_sensor_data)
    n.create_subscription(LaserScan, "/scan_safety",
                          lambda m: s.__setitem__("sc", (time.time(), m)), qos_profile_sensor_data)
    end = time.time() + 3.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

    fail = []
    st = s.get("st")
    if not st or not st.armed or st.mode != "GUIDED":
        fail.append(f"not armed+GUIDED (mode={st.mode if st else '?'}, "
                    f"armed={st.armed if st else '?'})")

    tg = s.get("tg")
    if not tg or time.time() - tg[0] > 0.8:
        fail.append("no fresh target -- stand in front of the rover, facing it")
    else:
        p = tg[1].pose.position
        rng, brg = math.hypot(p.x, p.y), math.degrees(math.atan2(p.y, p.x))
        if not (MIN_RANGE <= rng <= MAX_RANGE):
            fail.append(f"target at {rng:.2f} m, need {MIN_RANGE}-{MAX_RANGE} m "
                        f"(inside the deadband so it cannot lurch on enable)")
        if abs(brg) > MAX_BEARING_DEG:
            fail.append(f"target {brg:+.1f} deg off axis, need within "
                        f"+-{MAX_BEARING_DEG:.0f}")

    sc = s.get("sc")
    if not sc or time.time() - sc[0] > 0.5:
        fail.append("/scan_safety stale")
    else:
        m = sc[1]
        k = 0
        for i, r in enumerate(m.ranges):
            if math.isfinite(r) and m.range_min <= r <= m.range_max:
                a = m.angle_min + i * m.angle_increment
                if 0.85 <= r * math.cos(a) <= 1.45 and abs(r * math.sin(a)) <= 0.70:
                    k += 1
        if k > 3:
            fail.append(f"{k} points already in the STOP zone")

    if not fail and s.get("tg"):
        p = s["tg"][1].pose.position
        print(f"{time.strftime('%H:%M:%S')} GATE PASS  target "
              f"{math.hypot(p.x, p.y):.2f} m, {math.degrees(math.atan2(p.y, p.x)):+.1f} deg")
    else:
        print(f"{time.strftime('%H:%M:%S')} GATE FAIL: " + "; ".join(fail))
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if not fail else 1)


if __name__ == "__main__":
    main()
