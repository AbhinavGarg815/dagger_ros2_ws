#!/usr/bin/env python3
"""Static wiring check for the follow-me safety path. NON-MOTION: reads only.

    python3 tools/follow_check.py

Run with T1 + T2 + T4 (follow.launch.py) up. Verifies everything that can be
checked WITHOUT a command flowing:

    - collision_monitor_follow is lifecycle-ACTIVE
    - /scan_safety is live and fresh (the monitor's only obstacle source)
    - /local_costmap/published_footprint is live and non-empty -- the approach
      polygon takes its shape from this topic and, if it never arrives, stays
      empty and NEVER TRIGGERS with no warning logged (Humble polygon.cpp:314)
    - /cmd_vel_follow is published by exactly one node (the monitor) and has a
      subscriber (twist_mux), i.e. nothing bypasses the gate

The zone polygons themselves can NOT be checked here: the monitor only publishes
them from process(), which runs per incoming cmd_vel. Use tools/follow_gate_test.py
(user-run, rover disarmed) for that and for the live stop/slow behaviour.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import GetState

MONITOR = "collision_monitor_follow"
SAMPLE_S = 4.0


def main():
    rclpy.init()
    n = Node("follow_check")
    seen = {"scan": [], "fp": []}
    n.create_subscription(LaserScan, "/scan_safety",
                          lambda m: seen["scan"].append(time.time()), qos_profile_sensor_data)
    n.create_subscription(PolygonStamped, "/local_costmap/published_footprint",
                          lambda m: seen["fp"].append((time.time(), len(m.polygon.points))), 10)

    results = []

    def check(ok, name, detail=""):
        results.append((ok, name, detail))

    # --- lifecycle state -----------------------------------------------------
    cli = n.create_client(GetState, f"/{MONITOR}/get_state")
    if not cli.wait_for_service(timeout_sec=5.0):
        check(False, "monitor lifecycle", f"/{MONITOR}/get_state not available -- is T4 up?")
    else:
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
        label = fut.result().current_state.label if fut.result() else "?"
        check(label == "active", "monitor lifecycle", f"state={label}")

    end = time.time() + SAMPLE_S
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

    # --- obstacle source -----------------------------------------------------
    rate = len(seen["scan"]) / SAMPLE_S
    age = time.time() - seen["scan"][-1] if seen["scan"] else 999
    check(rate >= 8.0 and age < 0.5, "/scan_safety",
          f"{rate:.1f} Hz, age {age:.2f} s (want >=8 Hz, <0.5 s)")

    # --- approach polygon's shape source -------------------------------------
    if not seen["fp"]:
        check(False, "/local_costmap/published_footprint",
              "SILENT -- FootprintApproach will never trigger. Is T2 (nav2_gps) up?")
    else:
        pts = seen["fp"][-1][1]
        check(pts >= 3, "/local_costmap/published_footprint",
              f"{len(seen['fp']) / SAMPLE_S:.1f} Hz, {pts} vertices")

    # --- command path --------------------------------------------------------
    pubs = n.get_publishers_info_by_topic("/cmd_vel_follow")
    names = [f"{p.node_namespace.rstrip('/')}/{p.node_name}" for p in pubs]
    check(len(pubs) == 1 and pubs[0].node_name == MONITOR, "/cmd_vel_follow publishers",
          f"{names or 'none'} (want exactly [{MONITOR}] -- anything else BYPASSES the gate)")

    subs = n.get_subscriptions_info_by_topic("/cmd_vel_follow")
    sub_names = [s.node_name for s in subs]
    check("twist_mux" in sub_names, "/cmd_vel_follow subscribers",
          f"{sub_names or 'none'} (want twist_mux)")

    raw_pubs = n.get_publishers_info_by_topic("/cmd_vel_follow_raw")
    print(f"note: /cmd_vel_follow_raw publishers: "
          f"{[p.node_name for p in raw_pubs] or 'none (no follow controller yet)'}")

    print()
    for ok, name, detail in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:38s} {detail}")
    bad = [r for r in results if not r[0]]
    print(f"\n{time.strftime('%H:%M:%S')} follow_check "
          f"{'PASS' if not bad else f'FAIL ({len(bad)})'}")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if not bad else 1)


if __name__ == "__main__":
    main()
