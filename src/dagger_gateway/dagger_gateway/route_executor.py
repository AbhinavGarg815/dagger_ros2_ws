"""Drives a saved route with Nav2. MOTION: the owner tests it with the RC in hand.

GPS map mode (nav2_gps.launch.py: map = ArduPilot EK3 local ENU, map->odom from
gps_map_odom), exactly as tools/gps_goal.py does it:
    goal_map = local_now + ENU(goal_latlon - global_now)
SLAM map mode (map_source:=slam) is refused for now, with a log line: its
lat/lon -> map goes through the map's _geo.yaml (Phase 6).

The global costmap is a 40 m rolling window, so the route goes to Nav2 a window
at a time (routes.window: the next points within `window_m` of the rover) as one
NavigateThroughPoses goal; when it succeeds the next window follows. Nav2's
feedback (number_of_poses_remaining) says which points are passed.

Threads: start/pause/resume/cancel come from the asyncio loop; Nav2's callbacks
arrive in the rclpy thread. Every callback to the gateway goes through `post`
(loop.call_soon_threadsafe), so the core is only ever touched on the loop.
A generation number drops callbacks from a goal that was paused or cancelled.
"""
import math
import threading

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient

from .routes import ll_to_en, window


class RouteExecutor:

    def __init__(self, node, window_m, post, on_reached, on_done, on_failed):
        """on_reached(index) per point passed (0-based); on_done() at the end;
        on_failed(why) when Nav2 refuses or gives up (why: 'no_position',
        'map_mode', 'no_nav2', 'rejected', 'aborted')."""
        self.node, self.window_m, self.post = node, window_m, post
        self.on_reached, self.on_done, self.on_failed = on_reached, on_done, on_failed
        self.client = ActionClient(node, NavigateThroughPoses, "/navigate_through_poses")
        self.lock = threading.Lock()
        self.points, self.index, self.batch = [], 0, (0, 0)
        self.handle, self.gen = None, 0

    # ---------------- from the loop ----------------
    def start(self, points, index=0):
        with self.lock:
            self.points, self.index = list(points), index
        self._send_window()

    def resume(self):
        self._send_window()

    def pause(self):
        """Stops the rover where it is (Nav2 goal cancelled); the place in the route is kept."""
        with self.lock:
            self.gen += 1
            h, self.handle = self.handle, None
        if h is not None:
            h.cancel_goal_async()

    def cancel(self):
        self.pause()
        with self.lock:
            self.points, self.index = [], 0

    def precheck(self, points):
        """Before a route starts: the first waypoint (1-based) in the first window that
        sits on an obstacle with no clear spot within 1 m, or 0. Points beyond the
        window are checked as their window goes out (and skipped if blocked)."""
        w = self._window_poses(points, 0)
        if w is None:
            return 0
        s, _, xy, _ = w
        for k, (x, y) in enumerate(xy):
            if self.node.costmap_fix(x, y)[0] == "blocked":
                return s + k + 1
        return 0

    # ---------------- Nav2 ----------------
    def _window_poses(self, points, index):
        """(s, e, [(x, y) in map], here) for the next window, or None with no position."""
        fix = self.node.fresh("/mavros/global_position/global", 2.0)
        local = self.node.fresh("/mavros/local_position/odom", 1.0)
        if fix is None or local is None or not (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)):
            return None
        here = (fix.latitude, fix.longitude)
        lx, ly = local.pose.pose.position.x, local.pose.pose.position.y
        s, e = window(points, index, here, self.window_m)
        xy = []
        for lat, lon in points[s:e]:
            de, dn = ll_to_en(lat, lon, here[0], here[1])
            xy.append((lx + de, ly + dn))
        return s, e, xy, (lx, ly)

    def _map_mode_gps(self):
        return "gps_map_odom" in set(self.node.get_node_names())

    def _send_window(self):
        if not self._map_mode_gps():
            self.node.get_logger().error("route: map->odom is not gps_map_odom (SLAM mode?): "
                                         "routes need GPS map mode for now")
            self.post(self.on_failed, "map_mode")
            return
        if not self.client.server_is_ready():
            self.post(self.on_failed, "no_nav2")
            return
        with self.lock:
            w = self._window_poses(self.points, self.index)
            if w is not None:
                s, e, xy, prev = w
                self.batch = (s, e)
                self.gen += 1
                gen = self.gen
        if w is None:
            self.post(self.on_failed, "no_position")
            return
        # The waypoint check: a goal just inside an obstacle's margin moves to the
        # nearest clear spot (<= 1 m); one deeper in is skipped -- Nav2's planner
        # would refuse it and give up on the whole route.
        poses = []
        for k, (x, y) in enumerate(xy):
            verdict, x2, y2 = self.node.costmap_fix(x, y)
            if verdict == "blocked":
                self.node.get_logger().warn(f"route: point {s + k + 1} is on an obstacle -- skipped")
                continue
            if verdict == "moved":
                self.node.get_logger().info(f"route: point {s + k + 1} moved {math.hypot(x2 - x, y2 - y):.2f} m clear")
            poses.append(self._pose(x2, y2, math.atan2(y2 - prev[1], x2 - prev[0])))
            prev = (x2, y2)
        if not poses:
            self.post(self.on_failed, "aborted")
            return
        self.node.get_logger().info(f"route: points {s + 1}-{e} of {len(self.points)} to Nav2")
        fut = self.client.send_goal_async(NavigateThroughPoses.Goal(poses=poses),
                                          feedback_callback=lambda fb, g=gen: self._feedback(fb, g))
        fut.add_done_callback(lambda f, g=gen: self._accepted(f, g))

    def _accepted(self, fut, gen):
        gh = fut.result()
        with self.lock:
            if gen != self.gen:
                if gh is not None and gh.accepted:
                    gh.cancel_goal_async()      # paused while the goal was on its way
                return
            if gh is None or not gh.accepted:
                self.post(self.on_failed, "rejected")
                return
            self.handle = gh
        gh.get_result_async().add_done_callback(lambda r, g=gen: self._result(r, g))

    def _feedback(self, fb, gen):
        remaining = fb.feedback.number_of_poses_remaining
        self._passed_up_to(gen, self.batch[1] - remaining)

    def _passed_up_to(self, gen, upto):
        with self.lock:
            if gen != self.gen:
                return
            passed = list(range(self.index, max(self.index, upto)))
            self.index = max(self.index, upto)
        for i in passed:
            self.post(self.on_reached, i)

    def _result(self, fut, gen):
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._passed_up_to(gen, self.batch[1])
            with self.lock:
                if gen != self.gen:
                    return
                done = self.index >= len(self.points)
                if done:
                    self.handle = None
            if done:
                self.post(self.on_done)
            else:
                self._send_window()
        elif status == GoalStatus.STATUS_ABORTED:
            with self.lock:
                if gen != self.gen:
                    return
                self.handle = None
            self.post(self.on_failed, "aborted")
        # CANCELED: ours (pause, cancel, STOP) — nothing to report

    @staticmethod
    def _pose(x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = "map"
        # Stamp 0 = latest transform (tools/gps_goal.py, send_goal.py)
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p
