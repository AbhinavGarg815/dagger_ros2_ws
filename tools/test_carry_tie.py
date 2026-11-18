#!/usr/bin/env python3
"""The Earth tie carried across a restart of the map of this drive (map_tie.carry_fit). NO ROS.

    python3 tools/test_carry_tie.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "dagger_bringup"))
from dagger_bringup.map_tie import carry_fit, to_latlon  # noqa: E402


def main():
    old = {"lat0": 26.7480, "lon0": 75.8553, "rot": math.radians(20), "tx": 3.0, "ty": -2.0, "rms": 0.12}
    old_pose = (12.0, 5.0, math.radians(30))           # on the old map
    new_pose = (0.0, 0.0, 0.0)                         # a restart: the new map starts at the rover
    new = carry_fit(old, old_pose, new_pose)
    a, b = to_latlon(old, *old_pose[:2]), to_latlon(new, *new_pose[:2])
    d = math.hypot((a[0] - b[0]) * 111320, (a[1] - b[1]) * 111320 * math.cos(math.radians(a[0])))
    checks = [
        ("the rover is at the same spot on the Earth on both maps", d < 1e-3),
        ("and faces the same way on the Earth", abs((old["rot"] + old_pose[2]) - (new["rot"] + new_pose[2])) < 1e-9),
    ]
    # one metre ahead of the rover on the new map lands one metre ahead on the old map's Earth
    ahead_old = to_latlon(old, old_pose[0] + math.cos(old_pose[2]), old_pose[1] + math.sin(old_pose[2]))
    ahead_new = to_latlon(new, math.cos(new_pose[2]), math.sin(new_pose[2]))
    d2 = math.hypot((ahead_old[0] - ahead_new[0]) * 111320,
                    (ahead_old[1] - ahead_new[1]) * 111320 * math.cos(math.radians(a[0])))
    checks.append(("a point 1 m ahead lands in the same place", d2 < 1e-3))
    checks.append(("its error is the old one plus the restart's", abs(new["rms"] - 0.62) < 1e-9))
    new2 = carry_fit(old, old_pose, (4.0, -1.0, math.radians(-50)))      # after a jump: anywhere
    b2 = to_latlon(new2, 4.0, -1.0)
    d3 = math.hypot((a[0] - b2[0]) * 111320, (a[1] - b2[1]) * 111320 * math.cos(math.radians(a[0])))
    checks.append(("from any first pose on the new map", d3 < 1e-3))
    for what, ok in checks:
        print(("ok    " if ok else "FAIL  ") + what)
    bad = [c for c in checks if not c[1]]
    print("all passed" if not bad else f"{len(bad)} FAILED")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
