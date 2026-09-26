#!/usr/bin/env python3
"""Exit 0 only if FCU telemetry is actually arriving. NON-MOTION: reads only.

    python3 tools/fcu_flowing.py [seconds]

Used by follow_solo.sh before it enables follow. "The recorder has subscribed"
is not the same as "data is arriving": on 2026-09-25 (stop_test_173925) the bag
showed FCU topics starting 6 s after follow was enabled. This also catches the
streams being down entirely, e.g. after a mavros restart has reset the
SET_MESSAGE_INTERVAL rates that preflight configures.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import RCIn, RCOut

NEED = {"/mavros/rc/out": (RCOut, 10), "/mavros/rc/in": (RCIn, 3),
        "/mavros/local_position/velocity_body": (TwistStamped, 10)}


def main():
    window = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    n = Node("fcu_flowing")
    got = {k: 0 for k in NEED}
    for topic, (typ, _) in NEED.items():
        n.create_subscription(typ, topic, lambda m, k=topic: got.__setitem__(k, got[k] + 1),
                              qos_profile_sensor_data)
    # Wait for the FIRST message on every topic, not a fixed delay: a fresh
    # process takes ~6 s to discover mavros's publishers (measured 2026-09-26;
    # /mavros/state 8.5 s). The first version waited 2 s and then counted for 3,
    # so it listened entirely inside that dead window and reported healthy
    # telemetry as NOT FLOWING -- blocking a test on a false alarm.
    end = time.time() + 20.0
    while time.time() < end and not all(got.values()):
        rclpy.spin_once(n, timeout_sec=0.05)
    silent = [k for k in NEED if not got[k]]
    if silent:
        print("FCU telemetry NOT FLOWING: no message at all in 20 s on " + ", ".join(silent))
        n.destroy_node(); rclpy.shutdown(); sys.exit(1)
    for k in got:
        got[k] = 0
    end = time.time() + window
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    bad = [f"{k} {got[k] / window:.1f} Hz (need {NEED[k][1]})"
           for k in NEED if got[k] / window < NEED[k][1]]
    print("FCU telemetry " + ("FLOWING" if not bad else "NOT FLOWING: " + "; ".join(bad)))
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if not bad else 1)


if __name__ == "__main__":
    main()
