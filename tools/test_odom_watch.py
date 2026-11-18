#!/usr/bin/env python3
"""The map manager's odometry watch (dagger_bringup/odom_watch.py). NO ROS.

    python3 tools/test_odom_watch.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "dagger_bringup"))
from dagger_bringup.odom_watch import OdomWatch  # noqa: E402


def main():
    checks = []
    w = OdomWatch()
    t = 0.0
    for k in range(100):                          # 1 m/s, 50 Hz, a gentle turn: nothing
        t = k * 0.02
        assert w.step(t, t * 1.0, 0.0, 0.1 * t) is None
    checks.append(("driving at 1 m/s and turning gently: nothing", True))
    checks.append(("a 0.3 s gap with no jump: nothing", w.step(t + 0.3, (t + 0.3) * 1.0, 0.0, 0.1 * t) is None))
    t += 0.3
    checks.append(("a jump of 3 m in 0.02 s: jump", w.step(t + 0.02, 0.0, 0.0, 0.1 * t) == "jump"))
    t += 0.02
    checks.append(("a turn of 60 deg in 0.02 s: jump", w.step(t + 0.02, 0.0, 0.0, 0.1 * t + math.radians(60)) == "jump"))
    t += 0.02
    checks.append(("silent 1 s: not yet lost", w.check(t + 1.0) is None))
    checks.append(("silent 2.5 s: lost", w.check(t + 2.5) == "lost"))
    checks.append(("lost is said once", w.check(t + 3.0) is None))
    checks.append(("odometry again (from zero, a restart): back, not a jump", w.step(t + 6.0, 0.0, 0.0, 0.0) == "back"))
    checks.append(("then driving on: nothing", w.step(t + 6.02, 0.02, 0.0, 0.0) is None))
    for what, ok in checks:
        print(("ok    " if ok else "FAIL  ") + what)
    bad = [c for c in checks if not c[1]]
    print("all passed" if not bad else f"{len(bad)} FAILED")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
