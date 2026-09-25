#!/usr/bin/env python3
"""One-line-per-tick record of the whole follow-me chain. NON-MOTION: reads only.

    python3 tools/follow_status.py             # 2 Hz to the terminal
    python3 tools/follow_status.py -r 5 | tee ~/dagger_bags/follow_<name>.log

Every stage of the chain on one timestamped line, which is what the solo test
protocol needs: the operator is out at the rover, so the record has to be
readable afterwards rather than watched live.

    det   people the detector sees
    tgt   the selected target: range and bearing from the rover
    raw   what the follow controller asked for
    out   what collision_monitor_follow actually passed through
    gate  CLEAR / SLOW / STOP, inferred from raw vs out
    scan  age of /scan_safety (stale = cmd_vel_safety zeroes everything)

raw and out differing is the safety layer doing its job, not a fault.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--rate", type=float, default=2.0)
    a = ap.parse_args()

    rclpy.init()
    n = Node("follow_status")
    be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=1)
    s = {"det": (0.0, 0), "tgt": None, "raw": None, "out": None,
         "scan": 0.0, "en": None}
    n.create_subscription(Detection2DArray, "/follow/detections",
                          lambda m: s.__setitem__("det", (time.time(), len(m.detections))), 10)
    n.create_subscription(PoseStamped, "/follow/target",
                          lambda m: s.__setitem__("tgt", (time.time(), m)), be)
    n.create_subscription(Twist, "/cmd_vel_follow_raw",
                          lambda m: s.__setitem__("raw", (time.time(), m)), 10)
    n.create_subscription(Twist, "/cmd_vel_follow",
                          lambda m: s.__setitem__("out", (time.time(), m)), 10)
    n.create_subscription(LaserScan, "/scan_safety",
                          lambda m: s.__setitem__("scan", time.time()), be)
    n.create_subscription(Bool, "/follow/enable",
                          lambda m: s.__setitem__("en", m.data), 10)

    def fresh(entry, limit=0.7):
        return entry is not None and time.time() - entry[0] < limit

    print("time     en  det  tgt: rng   brg | raw:  v      w   | out:  v      w   | gate   scan")
    period = 1.0 / a.rate
    nxt = time.time()
    try:
        while rclpy.ok():
            while time.time() < nxt:
                rclpy.spin_once(n, timeout_sec=0.02)
            nxt += period

            ndet = s["det"][1] if time.time() - s["det"][0] < 0.7 else 0
            if fresh(s["tgt"]):
                p = s["tgt"][1].pose.position
                tgt = f"{math.hypot(p.x, p.y):5.2f} {math.degrees(math.atan2(p.y, p.x)):+6.1f}"
            else:
                tgt = "    -      -"
            raw = s["raw"][1] if fresh(s["raw"], 0.5) else None
            out = s["out"][1] if fresh(s["out"], 0.5) else None
            rs = f"{raw.linear.x:+6.3f} {raw.angular.z:+6.3f}" if raw else "     -      -"
            os_ = f"{out.linear.x:+6.3f} {out.angular.z:+6.3f}" if out else "     -      -"

            if raw is None:
                gate = "-"
            elif abs(raw.linear.x) < 1e-3 and abs(raw.angular.z) < 1e-3:
                # Nothing was asked for, so nothing was blocked. The monitor also
                # ceases publishing after stop_pub_timeout of continuous zero, so
                # absent output here means idle -- checking `out is None` first
                # mislabelled a holding-station rover as STOP.
                gate = "idle"
            elif out is None or abs(out.linear.x) < 1e-3 < abs(raw.linear.x):
                gate = "STOP"
            else:
                r = out.linear.x / raw.linear.x
                gate = "CLEAR" if r > 0.97 else ("SLOW" if abs(r - 0.4) < 0.05 else f"x{r:.2f}")

            # s["scan"] is 0.0 until the first scan lands; printing the age
            # then would print the epoch, which is noise in a saved log.
            if s["scan"] == 0.0:
                scan = "   -  no scan"
            else:
                age = time.time() - s["scan"]
                scan = f"{age:4.2f}s{'  STALE' if age > 0.5 else ''}"
            en = {True: "ON ", False: "off", None: " ? "}[s["en"]]
            print(f"{time.strftime('%H:%M:%S')} {en} {ndet:3d}  {tgt} | {rs} | {os_} | "
                  f"{gate:6s} {scan}", flush=True)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
