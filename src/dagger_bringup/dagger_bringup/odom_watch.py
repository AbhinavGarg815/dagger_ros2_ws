"""Odometry watch for SLAM (owner, 2026-11-18): when odometry is lost or jumps, SLAM placed by it
would draw the walls in the wrong place. No ROS here (tools/test_odom_watch.py)."""
import math


class OdomWatch:
    """Feed every odometry pose (step) and check now and then (check). Says "jump" when two poses
    close in time are far apart, "lost" when odometry has been silent too long, "back" on the first
    pose after that."""

    def __init__(self, lost_s=2.0, jump_m=1.5, jump_rad=0.5):
        self.lost_s, self.jump_m, self.jump_rad = lost_s, jump_m, jump_rad
        self.last = None               # (t, x, y, yaw)
        self.lost = False

    def step(self, t, x, y, yaw):
        last, self.last = self.last, (t, x, y, yaw)
        if self.lost:
            self.lost = False
            return "back"
        if last is None:
            return None
        dt = t - last[0]
        moved = math.hypot(x - last[1], y - last[2])
        turned = abs((yaw - last[3] + math.pi) % (2 * math.pi) - math.pi)
        if (dt < 0.5 and moved > self.jump_m) or (dt < 0.2 and turned > self.jump_rad):
            return "jump"
        return None

    def check(self, t):
        if not self.lost and self.last is not None and t - self.last[0] > self.lost_s:
            self.lost = True
            return "lost"
        return None
