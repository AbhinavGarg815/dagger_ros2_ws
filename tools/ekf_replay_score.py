#!/usr/bin/env python3
"""(Q77) Merge tools/ekf_replay.py outputs into a tools/lo_bag_to_bench.py file, so tools/lo_eval.py
scores the replayed EKF variants against the same direct-registration references as the live
/lidar_odom/odometry and /odometry/filtered.

    python tools/ekf_replay_score.py <bench.npz from lo_bag_to_bench> <replay_dir> <merged.npz>
    python tools/lo_eval.py <merged.npz>

Also prints the sanity check: V0 (the live set-up replayed) against the recorded /odometry/filtered.
"""
import glob, math, os, sys
import numpy as np

base, rdir, out = sys.argv[1:4]
B = dict(np.load(base, allow_pickle=True))
st = B["stamps"]
methods = list(B["methods"])
for f in sorted(glob.glob(os.path.join(rdir, "*.npz"))):
    name = os.path.basename(f)[:-4]
    R = np.load(f)["odom"]
    k = np.clip(np.searchsorted(R[:, 0], st), 0, len(R) - 1)
    P = np.tile(np.eye(4), (len(st), 1, 1))
    c, s = np.cos(R[k, 3]), np.sin(R[k, 3])
    P[:, 0, 0], P[:, 0, 1], P[:, 1, 0], P[:, 1, 1] = c, -s, s, c
    P[:, 0, 3], P[:, 1, 3] = R[k, 1], R[k, 2]
    B[f"pose_{name}"] = P; B[f"cpu_{name}"] = np.zeros(1); methods.append(name)
B["methods"] = np.array(methods)
np.savez(out, **B)
if "pose_V0" in B:
    live, v0 = B["pose_odom_filtered"], B["pose_V0"]
    dp = np.hypot(*(live[:, :2, 3] - v0[:, :2, 3]).T)
    dy = np.degrees(np.abs(np.angle(np.exp(1j * (np.arctan2(live[:, 1, 0], live[:, 0, 0]) - np.arctan2(v0[:, 1, 0], v0[:, 0, 0]))))))
    print(f"sanity: V0 replay vs recorded /odometry/filtered: position diff median {np.median(dp):.3f} m, max {dp.max():.3f} m; "
          f"yaw diff median {np.median(dy):.2f} deg, max {dy.max():.2f} deg")
print(f"merged -> {out}: methods {methods}")
