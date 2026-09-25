#!/usr/bin/env python3
"""Bench test for the follow-path collision monitor. RUN THIS ONLY DISARMED.

    python3 tools/follow_gate_test.py                  # 60 s, 0.3 m/s forward
    python3 tools/follow_gate_test.py --vx 0.3 -t 90

Stands in for the not-yet-written follow controller: publishes a steady command
on /cmd_vel_follow_raw and watches what comes out of collision_monitor_follow on
/cmd_vel_follow, so the stop and slow zones can be validated by walking in front
of the rover -- exactly how the Nav2 monitor was validated on 2026-09-17.

SAFETY. This publishes a velocity, so it REFUSES to start unless the FCU reports
DISARMED, and it stops publishing the moment the FCU arms mid-run. Disarmed, the
setpoints reach mavros and go nowhere. Keep the RC transmitter in hand anyway.

It also checks the thing follow_check.py cannot: the monitor only publishes its
zone polygons from process(), i.e. while a command is flowing, and an approach
polygon whose footprint topic never arrived publishes EMPTY rather than not at
all. A FootprintApproach point count of 0 here means that gate is dead.

Expected walk-in sequence, standing (not crouching -- the LiDAR cannot see below
~0.97 m at the bumper), approaching down the centre line:
    CLEAR -> SLOW (x0.4, from ~2.6 m) -> STOP (0, from ~1.45 m) -> CLEAR again
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, PolygonStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State

ZONES = {"PolygonStop": "/collision_monitor_follow/stop_zone",
         "PolygonSlow": "/collision_monitor_follow/slow_zone",
         "FootprintApproach": "/collision_monitor_follow/footprint"}


def classify(vx_in, vx_out, out_age, seen_output):
    # Before the monitor has published anything the output is absent, not zero.
    # Counting that as a stop lets the test pass on a startup artifact instead
    # of on the stop zone actually firing -- which is exactly what happened on
    # the first run (2026-09-25): 4/4 PASS while the closest approach was
    # 1.96 m, well outside the 1.45 m stop polygon.
    if not seen_output:
        return "waiting"
    if out_age > 0.5:
        return "STOP(no output)"
    if abs(vx_out) < 1e-3:
        return "STOP"
    r = vx_out / vx_in
    if r > 0.97:
        return "CLEAR"
    if abs(r - 0.4) < 0.05:
        return "SLOW"
    return f"APPROACH(x{r:.2f})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vx", type=float, default=0.3, help="forward command, m/s")
    ap.add_argument("--wz", type=float, default=0.0, help="yaw command, rad/s")
    ap.add_argument("-t", "--duration", type=float, default=60.0)
    a = ap.parse_args()

    rclpy.init()
    n = Node("follow_gate_test")
    s = {"state": None, "out": None, "out_t": 0.0, "scan": None}
    zone_pts = {k: None for k in ZONES}

    n.create_subscription(State, "/mavros/state",
                          lambda m: s.__setitem__("state", m), qos_profile_sensor_data)
    n.create_subscription(Twist, "/cmd_vel_follow",
                          lambda m: (s.__setitem__("out", m), s.__setitem__("out_t", time.time())), 10)
    n.create_subscription(LaserScan, "/scan_safety",
                          lambda m: s.__setitem__("scan", m), qos_profile_sensor_data)
    for name, topic in ZONES.items():
        n.create_subscription(PolygonStamped, topic,
                              lambda m, k=name: zone_pts.__setitem__(k, len(m.polygon.points)), 10)
    pub = n.create_publisher(Twist, "/cmd_vel_follow_raw", 10)

    # --- arm gate ------------------------------------------------------------
    end = time.time() + 5.0
    while time.time() < end and s["state"] is None:
        rclpy.spin_once(n, timeout_sec=0.05)
    st = s["state"]
    if st is None:
        print("REFUSED: no /mavros/state -- cannot confirm the rover is disarmed.")
        sys.exit(1)
    if st.armed:
        print(f"REFUSED: FCU is ARMED (mode={st.mode}). Disarm before running this test.")
        sys.exit(1)
    print(f"FCU disarmed (mode={st.mode}). Publishing {a.vx:.2f} m/s on /cmd_vel_follow_raw "
          f"for {a.duration:.0f} s.\n")
    print(f"{'time':8s} {'in':>6s} {'out':>6s}  {'gate':16s} {'nearest ahead':>14s}  zone vertices")

    cmd = Twist()
    cmd.linear.x, cmd.angular.z = a.vx, a.wz
    zero = Twist()
    seen = set()
    fp_max = 0
    seen_output = False
    nearest_ever = float("inf")
    t_end = time.time() + a.duration
    next_log = 0.0
    aborted = None
    try:
        while time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.02)
            if s["state"] is not None and s["state"].armed:
                aborted = "FCU ARMED mid-run"
                break
            pub.publish(cmd)

            now = time.time()
            if now < next_log:
                continue
            next_log = now + 0.5

            out = s["out"]
            vx_out = out.linear.x if out else 0.0
            seen_output = seen_output or out is not None
            gate = classify(a.vx, vx_out, now - s["out_t"], seen_output)
            seen.add(gate.split("(")[0])
            fp_max = max(fp_max, zone_pts["FootprintApproach"] or 0)

            near = float("inf")
            sc = s["scan"]
            if sc:
                for i, r in enumerate(sc.ranges):
                    if math.isfinite(r) and sc.range_min <= r <= sc.range_max:
                        ang = sc.angle_min + i * sc.angle_increment
                        if r * math.cos(ang) > 0.0 and abs(r * math.sin(ang)) <= 0.85:
                            near = min(near, r)
            nearest_ever = min(nearest_ever, near)
            vtx = " ".join(f"{k.replace('Polygon', '').replace('Footprint', 'Fp')}="
                           f"{zone_pts[k] if zone_pts[k] is not None else '-'}" for k in ZONES)
            print(f"{time.strftime('%H:%M:%S')} {a.vx:6.2f} {vx_out:6.2f}  {gate:16s} "
                  f"{near:12.2f} m  {vtx}")
    except KeyboardInterrupt:
        aborted = "Ctrl-C"
    finally:
        for _ in range(10):
            pub.publish(zero)
            rclpy.spin_once(n, timeout_sec=0.01)
            time.sleep(0.02)

    print()
    if aborted:
        print(f"ABORTED: {aborted} -- zeros published.")
    ok = []
    ok.append(("CLEAR passes the command through", "CLEAR" in seen))
    ok.append(("SLOW zone engaged (x0.4)", "SLOW" in seen))
    ok.append(("STOP zone engaged (0)", "STOP" in seen))
    print(f"  closest approach seen: {nearest_ever:.2f} m "
          f"(the stop polygon starts at 1.45 m from base_footprint, "
          f"i.e. ~0.6 m in front of the bumper)")
    ok.append(("FootprintApproach has a shape", fp_max >= 3))
    for name, good in ok:
        print(f"  [{'PASS' if good else 'FAIL'}] {name}")
    if fp_max < 3:
        print("  -> FootprintApproach is EMPTY: /local_costmap/published_footprint never "
              "arrived. That gate is dead. Is T2 (nav2_gps) up?")
    if "STOP" not in seen and nearest_ever > 1.45:
        print(f"  -> STOP never fired because nothing came closer than "
              f"{nearest_ever:.2f} m. Walk in to about 0.5 m from the front of the "
              f"rover -- the stop zone is much tighter than the slow zone.")
    elif not ("SLOW" in seen and "STOP" in seen):
        print("  -> SLOW/STOP need someone to walk into the zones; standing, down the "
              "centre line, from beyond 3 m.")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(g for _, g in ok) and not aborted else 1)


if __name__ == "__main__":
    main()
