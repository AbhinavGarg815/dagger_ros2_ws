"""Drives a saved route with Nav2. MOTION: the owner tests it with the RC in hand.

GPS map mode (nav2_gps.launch.py: map = ArduPilot EK3 local ENU, map->odom from
gps_map_odom), exactly as tools/gps_goal.py does it:
    goal_map = local_now + ENU(goal_latlon - global_now)
A MAP MISSION (Step D1) is driven on its map instead: its points are (x, y) on a
saved map the rover finds itself on (shadow localization, /slam/pose). For each
window, where the map lies in odom comes from the SLAM pose and the odom pose at
the same moment (routes.map_to_odom), and the goals go to Nav2 in `odom` -- where
it plans anyway -- so the GPS error and the GPS map frame play no part. Nav2 in
SLAM map mode (map_source:=slam) is still refused for GPS routes.
With GPS LOST (Step D3) a GPS route, or the way home, is driven on ODOMETRY: its
points converted once, from the last good GPS moment, to odom (routes.ll_to_odom /
the trail's own odom points), and sent to Nav2 in `odom` as they are.

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

from .routes import apply, ll_to_en, map_to_odom, stepping_stones, window


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
        self.xy = None                     # a map mission's points on its map, else None
        self.odom = None                   # the points in odom (GPS lost, Step D3), else None
        self.handle, self.gen = None, 0
        # Aiming at a step on a long leg (stepping_stones), not at a route point: reaching it passes
        # no point, and the next window goes out from there.
        self.stone = False

    # ---------------- from the loop ----------------
    def start(self, points, index=0, xy=None, odom=None):
        with self.lock:
            self.points, self.index = list(points), index
            self.xy = list(xy) if xy else None
            self.odom = list(odom) if odom else None
        self._send_window()

    @property
    def mode(self):
        """'map' (a map mission), 'odom' (driven on odometry, GPS lost) or 'gps'."""
        return "map" if self.xy else "odom" if self.odom else "gps"

    def to_odom(self, odom):
        """From now on drive the same points, given in odom (GPS lost); the place is kept."""
        with self.lock:
            self.odom = list(odom)

    def resume(self):
        self._send_window()

    def pause(self):
        """Stops the rover where it is (Nav2 goal cancelled); the place in the route is kept."""
        with self.lock:
            self.gen += 1
            h, self.handle = self.handle, None
        self.node.clear_plan()
        if h is not None:
            h.cancel_goal_async()

    def cancel(self):
        self.pause()
        with self.lock:
            self.points, self.index, self.xy, self.odom = [], 0, None, None

    def precheck(self, points, xy=None):
        """Before a route starts: the first waypoint (1-based) in the first window that
        sits on an obstacle with no clear spot within 1 m, or 0. Points beyond the
        window are checked as their window goes out (and skipped if blocked)."""
        w = self._window_poses(points, 0, xy, self.odom)
        if w is None:
            return 0
        s, _, goals, _, frame = w
        for k, (x, y) in enumerate(goals):
            if self.node.costmap_fix(x, y, frame)[0] == "blocked":
                return s + k + 1
        return 0

    # ---------------- Nav2 ----------------
    def _window_poses(self, points, index, xy=None, odom=None):
        """(s, e, [(x, y) goals], here, frame) for the next window, or None with no position.
        GPS routes: goals in `map` (the GPS map frame). Map missions (xy), and any route
        driven on odometry (odom): in `odom`."""
        if odom:
            now = self.node.odom_now()
            if now is None:
                return None
            s, e = window(odom, index, now[:2], self.window_m, dist=math.dist)
            return s, e, list(odom[s:e]), (now[0], now[1]), "odom"
        if xy:
            pair = self.node.map_pose_and_odom()
            now = self.node.odom_now()
            if pair is None or now is None:
                return None
            t = map_to_odom(*pair)
            s, e = window(xy, index, pair[0][:2], self.window_m, dist=math.dist)
            return s, e, [apply(t, x, y) for x, y in xy[s:e]], (now[0], now[1]), "odom"
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
        return s, e, xy, (lx, ly), "map"

    def _map_mode_gps(self):
        return "gps_map_odom" in set(self.node.get_node_names())

    def _send_window(self):
        if self.xy is None and self.odom is None and not self._map_mode_gps():
            self.node.get_logger().error("route: map->odom is not gps_map_odom (SLAM mode?): "
                                         "routes need GPS map mode for now")
            self.post(self.on_failed, "map_mode")
            return
        if not self.client.server_is_ready():
            self.post(self.on_failed, "no_nav2")
            return
        with self.lock:
            w = self._window_poses(self.points, self.index, self.xy, self.odom)
            stones = None
            if w is not None:
                s, e, xy, prev, frame = w
                stones = stepping_stones(prev, xy[0], self.window_m)
                if stones is not None:
                    e = s                      # no route point in this drive
                self.stone = stones is not None
                self.batch = (s, e)
                self.gen += 1
                gen = self.gen
                self.node.clear_plan()     # the plan drawn is for this drive from now on
        if w is None:
            self.post(self.on_failed, "no_position")
            return
        # The waypoint check: a goal just inside an obstacle's margin moves to the
        # nearest clear spot (<= 1 m); one deeper in is skipped -- Nav2's planner
        # would refuse it and give up on the whole route.
        poses = []
        if stones is not None:
            for x, y in stones:                # the first step not on an obstacle
                verdict, x2, y2 = self.node.costmap_fix(x, y, frame)
                if verdict != "blocked":
                    poses.append(self._pose(x2, y2, math.atan2(y2 - prev[1], x2 - prev[0]), frame))
                    break
            if poses:
                self.node.get_logger().info(
                    f"route: point {s + 1} is {math.dist(prev, xy[0]):.1f} m away, beyond the "
                    f"{self.window_m:g} m window: a step {math.dist(prev, (x2, y2)):.1f} m along the way to Nav2")
                fut = self.client.send_goal_async(NavigateThroughPoses.Goal(poses=poses),
                                                  feedback_callback=lambda fb, g=gen: self._feedback(fb, g))
                fut.add_done_callback(lambda f, g=gen: self._accepted(f, g))
                return
            # every step on an obstacle: the point itself, as before (Nav2 says if it cannot)
            with self.lock:
                self.stone = False
                self.batch = (s, s + 1)
            xy = xy[:1]
            e = s + 1
        for k, (x, y) in enumerate(xy):
            verdict, x2, y2 = self.node.costmap_fix(x, y, frame)
            if verdict == "blocked":
                self.node.get_logger().warn(f"route: point {s + k + 1} is on an obstacle -- skipped")
                continue
            if verdict == "moved":
                self.node.get_logger().info(f"route: point {s + k + 1} moved {math.hypot(x2 - x, y2 - y):.2f} m clear")
            poses.append(self._pose(x2, y2, math.atan2(y2 - prev[1], x2 - prev[0]), frame))
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
    def _pose(x, y, yaw, frame="map"):
        p = PoseStamped()
        p.header.frame_id = frame
        # Stamp 0 = latest transform (tools/gps_goal.py, send_goal.py)
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p
