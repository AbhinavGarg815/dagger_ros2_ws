#!/usr/bin/env python3
"""Free-space profile + route clearance in the ROBOT frame, read from the live costmap.

    python3 tools/route_check.py                     # just the lateral free bands ahead
    python3 tools/route_check.py "3,0 6,0.5 9,0.5"   # also score routes (send_goal --wp syntax)

Non-motion. Reads /global_costmap/costmap and TF (works whether the costmap is in
`odom` or `map`). Route points are forward,left metres from the rover, and each
segment is swept over a +-0.6 m corridor. Cost 99+ = lethal/inscribed (refuse),
50-98 = inflation margin (tolerable), 0 = clear.
"""
import math
import sys

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time


def main():
    routes = sys.argv[1:]
    rclpy.init()
    n = rclpy.create_node("route_check")
    got = {}
    n.create_subscription(
        OccupancyGrid, "/global_costmap/costmap", lambda m: got.__setitem__("g", m),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    for _ in range(120):
        rclpy.spin_once(n, timeout_sec=0.1)
        if "g" in got and buf.can_transform(got["g"].header.frame_id, "base_footprint", Time()):
            break
    else:
        raise SystemExit("no costmap or no TF to base_footprint")

    m = got["g"]
    t = buf.lookup_transform(m.header.frame_id, "base_footprint", Time()).transform
    rx, ry, q = t.translation.x, t.translation.y, t.rotation
    h = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(h), math.sin(h)
    W, H, r = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    a = np.array(m.data, dtype=np.int16).reshape(H, W)

    def cost(f, l):
        x, y = rx + c * f - s * l, ry + s * f + c * l
        i, j = int((y - oy) / r), int((x - ox) / r)
        return a[i, j] if 0 <= i < H and 0 <= j < W else -1

    print(f"costmap frame {m.header.frame_id}; rover ({rx:.2f},{ry:.2f}) yaw {math.degrees(h):.1f} deg")
    for fwd in (2, 4, 6, 8, 10, 12):
        free = [round(v, 1) for v in np.arange(-5, 5.01, 0.1) if 0 <= cost(fwd, v) < 50]
        if not free:
            print(f"  fwd {fwd:2d}: blocked")
            continue
        mid = min(free, key=abs)          # contiguous band around the centreline
        lo = hi = mid
        while round(lo - 0.1, 1) in free:
            lo = round(lo - 0.1, 1)
        while round(hi + 0.1, 1) in free:
            hi = round(hi + 0.1, 1)
        print(f"  fwd {fwd:2d}: free band {lo:+.1f} .. {hi:+.1f} (+=left)")

    for rt in routes:
        pts = [(0.0, 0.0)] + [tuple(map(float, p.split(","))) for p in rt.split()]
        segs = []
        for (f0, l0), (f1, l1) in zip(pts[:-1], pts[1:]):
            L = math.hypot(f1 - f0, l1 - l0)
            nf, nl = -(l1 - l0) / L, (f1 - f0) / L
            worst = max(cost(f0 + u * (f1 - f0) + off * nf, l0 + u * (l1 - l0) + off * nl)
                        for u in np.linspace(0, 1, int(L / 0.1) + 1)
                        for off in np.linspace(-0.6, 0.6, 5))
            segs.append(f"{math.degrees(math.atan2(l1 - l0, f1 - f0)):+.0f}deg/{worst}")
        print(f"route '{rt}': seg heading/maxcost  " + "  ".join(segs))


if __name__ == "__main__":
    main()
