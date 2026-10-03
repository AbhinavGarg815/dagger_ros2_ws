"""(Q73) Evaluate lo_bench.py output without ground truth.
   python lo_eval.py <bench.npz> [--dt 5]
Reference = direct GICP between two scans (0.1 m voxels, 50 iterations), started from every
method's own estimate and from odom; the best-fitting result is the reference (accepted when
>= 60 % of points are inliers). Three metrics, per method:
  RPE over dt s (every 0.5 s), translation (m) and yaw (deg);
  revisits: scans >= 30 s apart within 2 m -- the loop error accumulated in between;
  pivots: in-place turns (odom |wz| > 0.25, |vx| < 0.05, >= 2 s): error over the whole turn."""
import sys, math, argparse, numpy as np, small_gicp
ap = argparse.ArgumentParser(); ap.add_argument("npz"); ap.add_argument("--dt", type=float, default=5.0)
a = ap.parse_args()
D = np.load(a.npz, allow_pickle=True)
M = list(D["methods"]); st = D["stamps"]; kidx = list(D["keep_idx"]); keep = dict(zip(kidx, D["keep"]))
P = {m: D[f"pose_{m}"] for m in M}; od = D["odom"]


def odom_T(t):
    k = min(np.searchsorted(od[:, 0], t), len(od) - 1); x, y, h = od[k, 1:4]
    T = np.eye(4); T[:2, :2] = [[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]]; T[:2, 3] = (x, y); return T


def rel(m, i, j): return np.linalg.inv(P[m][i]) @ P[m][j]
def err(Tref, Test):
    E = np.linalg.inv(Tref) @ Test
    return float(np.linalg.norm(E[:2, 3])), float(abs(math.degrees(math.atan2(E[1, 0], E[0, 0]))))


def reference(i, j):
    inits = [rel(m, i, j) for m in M] + [np.linalg.inv(odom_T(st[i])) @ odom_T(st[j])]
    best = None
    for T0 in inits:
        r = small_gicp.align(keep[i].astype(np.float64), keep[j].astype(np.float64), T0, registration_type="GICP",
                             downsampling_resolution=0.1, max_correspondence_distance=1.0, max_iterations=50, num_threads=6)
        frac = r.num_inliers / max(len(keep[j]), 1)
        if best is None or (frac, -r.error) > (best[1], -best[2]):
            best = (r.T_target_source, frac, r.error)
    return best if best[1] >= 0.6 else None


def table(title, rows):
    print(f"\n{title}  ({len(rows)} pairs)")
    if not rows: return
    R = {m: np.array([r[m] for r in rows]) for m in M}
    print(f"  {'method':12s} {'trans median':>13s} {'p95':>7s} {'max':>7s} | {'yaw median':>10s} {'p95':>7s} {'max':>7s}")
    for m in M:
        t, y = R[m][:, 0], R[m][:, 1]
        print(f"  {m:12s} {np.median(t):13.3f} {np.percentile(t, 95):7.3f} {t.max():7.3f} | {np.median(y):10.2f} {np.percentile(y, 95):7.2f} {y.max():7.2f}")


name = a.npz.split("/")[-1]
print(f"{name}: {len(st)} scans, {st[-1] - st[0]:.0f} s; CPU ms/scan mean/p95: " +
      ", ".join(f"{m} {np.mean(D[f'cpu_{m}']):.0f}/{np.percentile(D[f'cpu_{m}'], 95):.0f}" for m in M))
for m in M:
    p = P[m][:, :2, 3]; print(f"  {m:10s} path {np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)):6.1f} m, z range {np.ptp(P[m][:, 2, 3]):.2f} m")
# 1. RPE
step = int(round(a.dt * 10))
rows, rejected = [], 0
for i in kidx:
    j = i + step
    if j not in keep: continue
    ref = reference(i, j)
    if ref is None: rejected += 1; continue
    rows.append({m: err(ref[0], rel(m, i, j)) for m in M})
table(f"RPE over {a.dt:.0f} s (reference rejected for {rejected})", rows)
worst = {m: max(rows, key=lambda r: r[m][0])[m] for m in M} if rows else {}
# 2. revisits
pos = np.array([P["vgicp" if "vgicp" in M else M[0]][i][:2, 3] for i in kidx])
loops, used = [], []
for ai, i in enumerate(kidx):
    for bj in range(ai + 1, len(kidx)):
        j = kidx[bj]
        if st[j] - st[i] < 30: continue
        if np.linalg.norm(pos[bj] - pos[ai]) < 2.0 and all(abs(st[i] - st[u]) > 15 or abs(st[j] - st[v]) > 15 for u, v in used):
            ref = reference(i, j)
            if ref is not None:
                loops.append({m: err(ref[0], rel(m, i, j)) for m in M}); used.append((i, j))
                print(f"   revisit {st[i]:6.1f} s <-> {st[j]:6.1f} s: " + ", ".join(f"{m} {loops[-1][m][0]:.2f} m/{loops[-1][m][1]:.1f}deg" for m in M))
table("Revisits (loop error accumulated between the two visits)", loops)
# 3. pivots
turn = (np.abs(od[:, 5]) > 0.25) & (np.abs(od[:, 4]) < 0.05)
segs, s0 = [], None
for k in range(len(od)):
    if turn[k] and s0 is None: s0 = k
    if not turn[k] and s0 is not None:
        if od[k - 1, 0] - od[s0, 0] >= 2.0: segs.append((od[s0, 0] - 0.5, od[k - 1, 0] + 0.5))
        s0 = None
piv = []
for ta, tb in segs:
    i = min(kidx, key=lambda k: abs(st[k] - ta)); j = min(kidx, key=lambda k: abs(st[k] - tb))
    ref = reference(i, j)
    if ref is None: print(f"   pivot {ta:.0f}-{tb:.0f} s: reference rejected"); continue
    yaw = math.degrees(math.atan2(ref[0][1, 0], ref[0][0, 0]))
    piv.append({m: err(ref[0], rel(m, i, j)) for m in M})
    jumps = {m: max(np.linalg.norm(P[m][k][:2, 3] - P[m][i][:2, 3]) for k in range(i, j + 1)) for m in M}
    print(f"   pivot {ta:6.1f}-{tb:6.1f} s, {yaw:+6.1f} deg (ref moved {np.linalg.norm(ref[0][:2, 3]):.2f} m): " +
          ", ".join(f"{m} err {piv[-1][m][0]:.2f} m/{piv[-1][m][1]:.1f}deg, max excursion {jumps[m]:.2f} m" for m in M))
table("Pivots (whole turn)", piv)
