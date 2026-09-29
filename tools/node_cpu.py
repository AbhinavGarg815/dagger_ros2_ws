#!/usr/bin/env python3
"""CPU % of running processes whose EXECUTABLE path/name contains a key.

    python3 tools/node_cpu.py localization_slam_toolbox_node slam_geo_init
    python3 tools/node_cpu.py -i 5 -d 600 -o ~/dagger_bags/cpu_loc2.csv localization_slam_toolbox_node

Matches only argv[0]/argv[1] (the program, or the script a python3 runs), never
the rest of the command line, and skips itself and all its ancestors: a pattern
in MY command line once matched my own shell (report H19), so a CPU logger
measured its wrapper and a kill hit my shell. PIDs are re-resolved every
interval, so a node restarted mid-run is followed.
"""
import argparse
import os
import time


def ancestors():
    out, pid = set(), os.getpid()
    while pid > 1:
        out.add(pid)
        try:
            pid = int(open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
    return out


def find(key, skip):
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) in skip:
            continue
        try:
            argv = open(f"/proc/{d}/cmdline", "rb").read().split(b"\0")
        except OSError:
            continue
        prog = [a.decode(errors="replace") for a in argv[:2]]
        if any(key in os.path.basename(a) for a in prog if a):
            return int(d)
    return None


def ticks(pid):
    try:
        f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
        return int(f[11]) + int(f[12])
    except (OSError, IndexError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("keys", nargs="+")
    ap.add_argument("-i", "--interval", type=float, default=10.0)
    ap.add_argument("-d", "--duration", type=float, default=0.0, help="0 = one sample")
    ap.add_argument("-o", "--out", help="append CSV rows here")
    a = ap.parse_args()
    hz = os.sysconf("SC_CLK_TCK")
    skip = ancestors()
    out = open(os.path.expanduser(a.out), "a") if a.out else None
    end = time.time() + a.duration
    prev = {}
    for k in a.keys:
        p = find(k, skip)
        prev[k] = (p, ticks(p) if p else None, time.time())
    while True:
        time.sleep(a.interval)
        now = time.time()
        row = []
        for k in a.keys:
            p = find(k, skip)
            t = ticks(p) if p else None
            pp, pt, pn = prev[k]
            if p and p == pp and t is not None and pt is not None:
                pct = 100.0 * (t - pt) / hz / (now - pn)
                row.append(f"{k}={pct:.1f}%(pid {p})")
                if out:
                    out.write(f"{now:.0f},{k},{p},{pct:.1f}\n")
            else:
                row.append(f"{k}=n/a")
            prev[k] = (p, t, now)
        print(time.strftime("%H:%M:%S"), "  ".join(row), flush=True)
        if out:
            out.flush()
        if now >= end:
            break


if __name__ == "__main__":
    main()
