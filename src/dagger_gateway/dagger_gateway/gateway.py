#!/usr/bin/env python3
"""The operator app's gateway: the only thing the app talks to.

    ros2 run dagger_gateway gateway                  # ws://<rover>:8090/api
    ros2 run dagger_gateway gateway --stop-dry-run   # STOP changes the state only

One WebSocket per app, binary frames, one protobuf Envelope each; the contract
is proto/ (git submodule dagger_proto) -- read proto/README.md and the header of
proto/dagger/api.proto. The gateway owns the operating state, readiness, alerts,
the link-loss policy and what commands are accepted; the app shows and asks.
Video (Phase 4) gets its own WebSocket, never /api, so a frame never delays a STOP.

Built like dagger_bringup/ui_server.py: rclpy spins in a background thread,
tornado (apt python3-tornado 6.1) owns the asyncio loop in the main thread,
crossings go through call_soon_threadsafe. (Not python3-websockets: Ubuntu
22.04's 9.1 never completes a connection on Python 3.10 -- see ui_server.py.)

Ports in use elsewhere: 8080 follow page, 8081 round-trip test, 8082 fake rover
(PC), 8765 Foxglove.
"""
import asyncio
import collections
import math
import signal
import sys
import threading
import time

import rclpy
import tornado.web
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.utilities import remove_ros_args

from . import api_pb2 as pb
from .ros_io import RosIO
from .route_executor import RouteExecutor
from .rover_state import RoverCore, Settings
from .routes import (HomeStore, MapTrail, Recorder, RouteStore, Trail, distance_m, home_path, ll_to_odom,
                     map_to_odom, mission_waypoints,
                     odom_to_ll, odom_to_map, path_length, progress)
from .gps_watch import DisagreeWatch, GpsWatch, RtkWatch
from .base import base_pb, sites_pb
from .maps import (MapPicture, from_picture, map_info, map_list_pb, mapping_pb, picture_heading, placed_map,
                   to_latlon, to_map, to_picture)
from .session import ApiSocket
from .video import VideoSocket
from .follow import follow_status, is_selectable, stamp_s, targets_msg, geometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

try:
    from .build_info import VERSION
except ImportError:                        # running from a tree setup.py has not built
    VERSION = "gateway (unbuilt)"

CAPABILITIES = (pb.CAPABILITY_STOP, pb.CAPABILITY_READINESS, pb.CAPABILITY_ALERTS)
ROUTE_CAPABILITIES = (pb.CAPABILITY_RUN_ROUTE, pb.CAPABILITY_RECORD_ROUTE, pb.CAPABILITY_PLAN_ROUTE,
                      pb.CAPABILITY_IMPORT_MISSION, pb.CAPABILITY_COME_HOME, pb.CAPABILITY_FOLLOW,
                      pb.CAPABILITY_SET_HOME)
IMPORT_WAIT_S = 4.0     # ImportMission is answered within this (the app waits 5 s)
MAP_WAIT_S = 3.0        # map commands: the map manager answers within this, or no answer
# The map manager's words for "no" -> the contract's reasons.
MAP_ERRORS = {"unknown map": pb.REJECT_REASON_UNKNOWN_MAP, "not mapping": pb.REJECT_REASON_NOT_IN_THIS_STATE,
              "already mapping": pb.REJECT_REASON_NOT_IN_THIS_STATE, "name taken": pb.REJECT_REASON_NAME_TAKEN,
              "nothing mapped": pb.REJECT_REASON_TOO_SHORT,
              "unknown site": pb.REJECT_REASON_UNKNOWN_SITE, "not surveyed": pb.REJECT_REASON_NOT_IN_THIS_STATE}
MAP_COMMANDS = ("show_map", "delete_map", "start_mapping", "save_map", "discard_mapping")
BASE_COMMANDS = ("survey_base", "use_base_site", "save_base_site", "delete_base_site")
# While one of these runs the base must not change: the rover's position would jump.
JOB_STATES = (pb.OPERATING_STATE_RUNNING_ROUTE, pb.OPERATING_STATE_PAUSED, pb.OPERATING_STATE_FOLLOWING,
              pb.OPERATING_STATE_COMING_HOME, pb.OPERATING_STATE_RECORDING, pb.OPERATING_STATE_MAPPING)


class Gateway:
    """asyncio side: the sessions, the core, the timers."""

    def __init__(self, node, loop):
        self.node, self.loop = node, loop
        self.log = node.get_logger()
        g = node.param
        self.rover_id, self.rover_name = g("rover_id"), g("rover_name")
        self.software_version = VERSION
        self.routes_on = bool(g("routes_enabled"))
        self.maps_on = bool(g("maps_enabled"))
        self.capabilities = (CAPABILITIES + (ROUTE_CAPABILITIES if self.routes_on else ())
                             + ((pb.CAPABILITY_MAPS, pb.CAPABILITY_FIND_ON_MAP, pb.CAPABILITY_HOME_ON_MAP)
                                if self.maps_on else ())
                             + ((pb.CAPABILITY_MAKE_MAP,) if self.maps_on and g("make_maps_enabled") else ())
                             + ((pb.CAPABILITY_BASE_STATION,) if g("base_enabled") else ()))
        self.base_on = bool(g("base_enabled"))
        node.on_base_sites = lambda: loop.call_soon_threadsafe(self.broadcast_base_sites)
        self.map_calls = set()                     # ShowMap / DeleteMap ids being answered
        node.on_maps = lambda: loop.call_soon_threadsafe(self.broadcast_maps)
        node.on_mapping = lambda: loop.call_soon_threadsafe(self.on_mapping_status)
        self.settings = Settings(heartbeat_ms=int(g("heartbeat_period_ms")),
                                 state_ms=int(g("state_period_ms")),
                                 follow_stop_ms=int(g("follow_stop_after_silence_ms")),
                                 route_pause_ms=int(g("route_pause_after_silence_ms")),
                                 max_speed_mps=float(g("max_speed_mps")),
                                 record_spacing_m=float(g("record_point_spacing_m")))
        self.sockets = set()
        self.results = collections.OrderedDict()   # command_id -> CommandResult (rule 3)
        self.importing = set()                     # ImportMission ids being read
        self.trail = None                          # routes.Trail from home, once the first job starts
        # Home kept across restarts (SetHome, owner 2026-10-23): the trail back starts from it.
        self.homes = HomeStore(g("home_file"))
        # Home on a saved map (indoors, Step D2+): (map name, x, y on the map), and where the
        # rover has been on the map in use, to drive home on it. One home at a time.
        self.map_home = None
        self.map_trail = None
        kept = self.homes.load()
        if kept is not None:
            self.trail = Trail(kept)
            self.log.info(f"home kept from before: {kept[0]:.7f}, {kept[1]:.7f}")
        else:
            self.map_home = self.homes.load_map()
            if self.map_home is not None:
                self.log.info(f"home kept from before, on the map '{self.map_home[0]}'")
        # Step D3, GPS lost during a job (DaggerGCS docs/proposals/operating-modes.md section 4)
        self.gps = GpsWatch(float(g("gps_lost_after_s")), float(g("gps_lost_accuracy_m")),
                            float(g("gps_back_after_s")))
        self.rtk_watch = RtkWatch(float(g("gps_lost_after_s")))
        self.disagree = DisagreeWatch(float(g("gps_lost_after_s")), float(g("position_disagree_deg")),
                                      float(g("position_disagree_m")))
        self.lidar_limit_m = float(g("lidar_carry_on_m"))
        self.snaps = collections.deque(maxlen=15)  # (monotonic s, gps_snapshot), 1 a second while GPS is good
        self.lost_snap = None                      # the last good one before GPS went
        self.lidar_m, self.lidar_last = 0.0, None  # driven on odometry since the last answer
        self.cube_source = 1                       # the EKF source set in use (GPS at boot)
        self.switching = False
        # Step D2: the INDOOR switch moves the Cube's source too (the last move wins); the
        # gateway switches by itself, when allowed, whenever there is no GPS and it is parked.
        self.indoor_switch = int(g("indoor_switch_channel"))
        self.switch_pos = None                     # the INDOOR switch's last position (1, 2, 3)
        self.switch_retry_at = 0.0                 # monotonic s: a refused switch is asked again after
        self.map_mission_gps = False               # GPS was good when the running map mission began
        self.drive_source = None                   # (status, values) of CHECK_DRIVE_SOURCE
        self.video_sockets = set()                 # apps watching the picture (video.VideoSocket)
        node.on_video = lambda raw: loop.call_soon_threadsafe(self.on_video_frame, raw)
        node.on_targets = lambda: loop.call_soon_threadsafe(self.on_targets)
        self.checks_in = None
        omit = () if g("battery_monitor") else (pb.CHECK_BATTERY,)
        self.core = RoverCore(self.settings, self.broadcast, self, omit_checks=omit)
        node.on_estop_edge = lambda on: loop.call_soon_threadsafe(self.on_estop_edge, on)
        # Phase 3: saved routes and the Nav2 executor (its callbacks come back on this loop)
        self.store = RouteStore(g("route_dir")) if self.routes_on else None
        post = lambda fn, *a: loop.call_soon_threadsafe(fn, *a)  # noqa: E731
        self.route_exec = RouteExecutor(node, float(g("route_window_m")), post,
                                        on_reached=self.core.route_reached,
                                        on_done=self.on_route_done,
                                        on_failed=self.on_route_failed) if self.routes_on else None

    # ---------------- actions for the core (link-loss policy) ----------------
    def follow_off(self, why):
        self.node.follow_off(why)

    def pause_route(self, why):
        if self.route_exec is not None:
            self.route_exec.pause()
        self.log.warn(f"route paused ({why})")

    def apps(self):
        return [s for s in self.sockets if s.accepted]

    def broadcast(self, ev):
        for s in self.apps():
            s.send_event(ev)

    # ---------------- commands ----------------
    def command(self, cmd, sock):
        kind = cmd.WhichOneof("kind")
        ok = pb.CommandResult(command_id=cmd.command_id, outcome=pb.COMMAND_OUTCOME_ACCEPTED)
        if kind == "stop":
            self.log.warn(f"STOP from app {sock.request.remote_ip} ({cmd.command_id})")
            if self.route_exec is not None:
                self.route_exec.cancel()   # our own route ends even in --stop-dry-run
            self.node.stop()
            self.core.stop()
            return ok
        if kind == "import_mission" and self.routes_on:
            self.import_mission(cmd, sock)
            return None                    # answered once the autopilot's mission is read
        if (kind in MAP_COMMANDS and self.maps_on) or (kind in BASE_COMMANDS and self.base_on):
            self.map_command(kind, cmd, sock)
            return None                    # answered once the map manager has done it
        handler = getattr(self, "on_" + kind, None) if (kind and self.routes_on) else None
        why = handler(getattr(cmd, kind)) if handler else pb.REJECT_REASON_NOT_SUPPORTED
        blocked = 0
        if isinstance(why, tuple):                 # (reason, which waypoint)
            why, blocked = why
        if why is None:
            self.log.info(f"{kind} from app {sock.request.remote_ip}: accepted")
            return ok
        self.log.info(f"{kind} from app {sock.request.remote_ip}: {pb.RejectReason.Name(why)}"
                      + (f" (waypoint {blocked})" if blocked else ""))
        return pb.CommandResult(command_id=cmd.command_id, outcome=pb.COMMAND_OUTCOME_REJECTED,
                                reject_reason=why, blocked_waypoint=blocked)

    def set_home(self):
        """The session's first job fixes home where the rover is (a 3-D fix)."""
        if self.trail is None and self.map_home is None:
            here = self.node.good_fix()
            if here is not None:
                self.trail = Trail(here)
                self.homes.save(*here)
                self.log.info(f"home set at {here[0]:.7f}, {here[1]:.7f}")
            else:
                f = self.found_pose()                    # indoors: where it is on the map in use
                if f is not None:
                    self.set_map_home(f[0], f[1], f[2])

    def found_pose(self):
        """(map name, x, y, yaw) of the rover on the shown map once found on it, else None."""
        ml, pose = self.node.maps, self.node.slam_pose
        if not (self.maps_on and ml is not None and ml.localizing and self.node.slam_localized and pose):
            return None
        return ml.localizing, pose[1], pose[2], pose[3]

    def set_map_home(self, name, x, y):
        """Home on a saved map (Step D2+). The map trail starts again from it."""
        self.map_home, self.trail = (name, x, y), None
        self.map_trail = MapTrail(name, (x, y))
        self.homes.save_map(name, x, y)
        self.log.info(f"home set on the map '{name}' at ({x:.2f}, {y:.2f})")

    def map_way_home(self):
        """(reject reason or None, points on the map or None): the way home on the map in use,
        when the rover is found on it and home is on it -- a home on this map, or a GPS home
        on a placed map (through its tie). (None, None): not on a map, go on GPS as before."""
        f = self.found_pose()
        target = None
        if self.map_home is not None:
            if f is None or f[0] != self.map_home[0]:
                return pb.REJECT_REASON_NOT_ON_MAP, None
            target = self.map_home[1:]
        elif self.trail is not None and f is not None:
            m = placed_map(self.node.maps, f[0])
            if m is not None:
                target = to_map(tuple(m.geo), *self.trail.home)
        if target is None:
            return None, None
        been = list(self.map_trail.points) if self.map_trail is not None and self.map_trail.name == f[0] else []
        if not been or math.dist(been[0], target) > 0.05:
            been = [tuple(target)] + been
        return None, home_path(been, (f[1], f[2]), dist=math.dist)

    def on_set_home(self, cmd):
        """Move home (owner, 2026-10-23): where the rover stands -- its trail back starts again
        here -- or a place tapped on the map: the way home then goes back along the trail as
        far as it leads and Nav2 plans the last leg. Moves nothing; kept across restarts."""
        if self.core.route is not None:
            return pb.REJECT_REASON_NOT_IN_THIS_STATE     # a mission or the way home is on
        if cmd.here:
            at = self.node.good_fix()
            if at is None:
                f = self.found_pose()                    # indoors: where it is on the map in use
                if f is None:
                    return pb.REJECT_REASON_NOT_READY
                self.set_map_home(f[0], f[1], f[2])
                return None
            odom = self.node.odom_now()
            self.trail = Trail(at, odom[:2] if odom is not None else None)
        elif cmd.on_map:
            m = map_info(self.node.maps, cmd.map_name)
            if m is None:
                return pb.REJECT_REASON_UNKNOWN_MAP
            if self.node.maps.localizing != cmd.map_name:
                return pb.REJECT_REASON_NOT_ON_MAP
            self.set_map_home(cmd.map_name, *from_picture(m, cmd.map_x_m, cmd.map_y_m))
            return None
        else:
            at = (cmd.at.latitude_deg, cmd.at.longitude_deg)
            if not (math.isfinite(at[0]) and math.isfinite(at[1])) or at == (0.0, 0.0):
                return pb.REJECT_REASON_NOT_SUPPORTED
            snap = self.node.gps_snapshot()
            self.trail = Trail(at, ll_to_odom([at], snap)[0] if snap is not None else None)
        self.map_home = None                             # one home: this one, on the Earth
        self.homes.save(*at)
        self.log.info(f"home moved {'to where the rover stands' if cmd.here else 'to a place on the map'}: "
                      f"{at[0]:.7f}, {at[1]:.7f}")
        return None

    def on_route_done(self):
        if self.core.going_home and (self.trail is not None or self.map_home is not None):
            if self.trail is not None:
                self.trail.reset()
            self.map_trail = MapTrail(*self.map_home[:1], self.map_home[1:]) if self.map_home else None
            self.log.info("route: home reached")
        self.core.route_finished()

    # Phase 3 (docs/proposals/phase3-routes.md in DaggerGCS): each returns None when
    # accepted, else the reject reason. The rules are the core's; the ROS side is here.
    def on_start_route(self, cmd):
        route = self.store.get(cmd.route_id)
        # A map mission (Step D2) is its own job: on the map, not GPS.
        why = self.core.may_start(pb.JOB_RUN_MAP_MISSION if route is not None and route.map_name
                                  else pb.JOB_RUN_ROUTE)
        if why:
            return why
        if route is None:
            return pb.REJECT_REASON_UNKNOWN_ROUTE
        if route.map_name:
            # A map mission: its map must be the one the rover is finding itself on, and found.
            ml = self.node.maps
            if not (ml is not None and ml.localizing == route.map_name and self.node.slam_localized):
                return pb.REJECT_REASON_NOT_ON_MAP
        if not self.node.armed():
            return pb.REJECT_REASON_NOT_ARMED
        # A map mission is driven and measured on its map: its (x, y), from the SLAM pose.
        pts = route.xy if route.map_name else route.points
        blocked = self.route_exec.precheck(pts, route.xy)
        if blocked:
            return pb.REJECT_REASON_WAYPOINT_BLOCKED, blocked
        self.set_home()
        if route.map_name:
            pose = self.node.slam_pose
            here = pose[1:3] if pose is not None else None
            total = (math.dist(here, pts[0]) if here else 0.0) + path_length(pts, math.dist)
            self.map_mission_gps = not self.gps.lost
        else:
            here = self.node.good_fix()
            total = (distance_m(here, pts[0]) if here else 0.0) + path_length(pts)
        self.node.release_estop()
        self.node.ensure_guided()
        self.core.route_started(route.id, pts, total)
        self.route_exec.start(pts, 0, route.xy)
        return None

    def on_set_here_on_map(self, cmd):
        """"I am here" (Step D2): the operator's tap on the shown map's picture, to the map
        frame, then to slam_geo_init, which searches around it. Moves nothing."""
        if not self.maps_on:
            return pb.REJECT_REASON_NOT_SUPPORTED
        m = map_info(self.node.maps, cmd.map_name)
        if m is None:
            return pb.REJECT_REASON_UNKNOWN_MAP
        if self.node.maps.localizing != cmd.map_name:
            return pb.REJECT_REASON_NOT_ON_MAP
        if self.core.route == "running" or self.core.following:
            return pb.REJECT_REASON_NOT_IN_THIS_STATE
        x, y = from_picture(m, cmd.x_m, cmd.y_m)
        yaw = math.atan2(cmd.front_y_m - cmd.y_m, cmd.front_x_m - cmd.x_m) if cmd.has_front else None
        self.node.send_here(x, y, yaw)
        self.log.info(f"I am here on '{cmd.map_name}': ({cmd.x_m:.2f}, {cmd.y_m:.2f}) on the picture"
                      + (f", facing {picture_heading(yaw):.0f} deg" if yaw is not None else ""))
        return None

    # ---------------- following (Phase 4) ----------------
    def on_video_frame(self, raw):
        """A camera frame (still serialized): decoded only when someone watches."""
        if not self.video_sockets:
            return
        img = deserialize_message(raw, CompressedImage)
        w, h, rot = geometry(*self.node.latest_tracks(5.0))
        frame = pb.VideoFrame(stamp_s=stamp_s(img.header), width=w, height=h, rotation_deg=rot,
                              jpeg=bytes(img.data))
        for s in list(self.video_sockets):
            s.send_frame(frame)

    def on_targets(self):
        if not self.video_sockets:
            return
        t = targets_msg(*self.node.latest_tracks())
        for s in list(self.video_sockets):
            s.send_targets(t)

    def on_select_target(self, cmd):
        """Pick who to follow (or -1: let go). Moves nothing, so any state."""
        if cmd.target_id >= 0 and not is_selectable(*self.node.latest_tracks(), cmd.target_id):
            return pb.REJECT_REASON_NO_TARGET
        self.node.select_target(cmd.target_id)
        return None

    def on_start_follow(self, _):
        why = self.core.may_start(pb.JOB_FOLLOW)
        if why:
            return why
        if not self.node.armed():
            return pb.REJECT_REASON_NOT_ARMED
        st = follow_status(*self.node.latest_tracks())
        if st is None or st.state == pb.TARGET_STATE_LOST:
            return pb.REJECT_REASON_NO_TARGET
        self.set_home()
        self.node.release_estop()
        self.node.ensure_guided()
        self.node.follow_on()          # FOLLOWING comes from /follow/state, as for the web page
        return None

    def on_stop_follow(self, _):
        if not self.core.following:
            return pb.REJECT_REASON_NOT_IN_THIS_STATE
        self.node.follow_off("app: Stop following")
        self.core.set_following(False)
        return None

    def on_come_home(self, _):
        """Return to Home: back along the trail to where the session's first job started.
        Also the answer to "GPS lost" (Step D3): the paused route ends and the rover goes
        home -- on odometry, along the trail's odom points, while GPS is lost."""
        gps_question = self.core.route == "paused" and self.core.pause_reason == pb.STATE_REASON_GPS_LOST
        why = None if gps_question else self.core.may_start(pb.JOB_COME_HOME)
        if why:
            return why
        if self.trail is None and self.map_home is None:
            return pb.REJECT_REASON_NO_HOME
        if not self.node.armed():
            return pb.REJECT_REASON_NOT_ARMED
        # Found on the map in use (Step D2+, owner 2026-10-23): home is driven on the map.
        why, points = self.map_way_home()
        if why:
            return why
        if points:
            if gps_question:
                self.route_exec.cancel()
                self.core.route_cancelled()
            pose = self.node.slam_pose
            total = math.dist(pose[1:3], points[0]) + path_length(points, math.dist)
            self.log.info(f"return to home on the map '{self.node.maps.localizing}': {len(points)} points, {total:.0f} m")
            self.node.release_estop()
            self.node.ensure_guided()
            self.core.route_started("", points, total, home=True)
            self.route_exec.start(points, 0, xy=points)
            return None
        if self.gps.lost:
            return self.come_home_on_odometry(gps_question)
        if gps_question:
            self.route_exec.cancel()
            self.core.route_cancelled()
        here = self.node.good_fix()
        if here is None:
            return pb.REJECT_REASON_NOT_READY
        points = home_path(self.trail.points, here)
        total = distance_m(here, points[0]) + path_length(points)
        self.log.info(f"return to home: {len(points)} points, {total:.0f} m")
        self.node.release_estop()
        self.node.ensure_guided()
        self.core.route_started("", points, total, home=True)
        self.route_exec.start(points, 0)
        return None

    def come_home_on_odometry(self, gps_question):
        odom = self.node.odom_now()
        if odom is None or len(self.trail.odom) < 1 or self.lost_snap is None:
            return pb.REJECT_REASON_NOT_READY
        if gps_question:
            self.route_exec.cancel()
            self.core.route_cancelled()
        xy = home_path(self.trail.odom, odom[:2], dist=math.dist)
        points = [odom_to_ll(x, y, self.lost_snap) for x, y in xy]
        here = odom_to_ll(odom[0], odom[1], self.lost_snap)
        total = distance_m(here, points[0]) + path_length(points)
        self.log.warn(f"return to home with no GPS, on odometry: {len(points)} points, {total:.0f} m")
        self.lidar_m = 0.0
        self.core.route_started("", points, total, home=True)

        def go(ok):
            if self.core.route != "running" or not self.core.going_home:
                return
            if not ok:                         # the Cube kept GPS: no position to drive on
                self.core.gps_ask_lidar_m = None
                self.core.route_paused(pb.STATE_REASON_GPS_LOST)
                return
            self.node.release_estop()
            self.node.ensure_guided()
            self.route_exec.start(points, 0, odom=xy)
        self.switch_source(2, go)
        return None

    def on_pause_route(self, _):
        if self.core.route != "running":
            return pb.REJECT_REASON_NOT_IN_THIS_STATE
        self.route_exec.pause()
        self.core.route_paused(pb.STATE_REASON_OPERATOR)
        return None

    def on_resume_route(self, _):
        if self.core.route != "paused" or self.core.stopped:
            return pb.REJECT_REASON_NOT_IN_THIS_STATE
        if not self.node.armed():
            return pb.REJECT_REASON_NOT_ARMED
        if self.gps.lost:
            # Carry on with no GPS (Step D3): a GPS route goes on, on odometry, from the
            # last good moment; a new allowance of lidar_carry_on_m before it asks again.
            if self.cube_source != 2 and not self.node.param("ekf_source_switching"):
                # The Cube stays on GPS (no position): only the INDOOR switch can change that.
                return pb.REJECT_REASON_NOT_READY
            if self.route_exec.mode == "gps":
                if self.lost_snap is None:
                    return pb.REJECT_REASON_NOT_READY
                self.route_exec.to_odom(ll_to_odom(self.core.route_points, self.lost_snap))
                self.log.warn("GPS lost: carrying on, on odometry")
            self.lidar_m, self.core.gps_ask_lidar_m = 0.0, None
            if self.cube_source != 2:
                def go(ok):
                    if self.core.route != "paused":
                        return
                    if ok:
                        self.resume_now()
                self.switch_source(2, go)
                return None
        self.resume_now()
        return None

    def resume_now(self):
        self.node.release_estop()
        self.node.ensure_guided()
        self.core.route_resumed()
        self.route_exec.resume()

    def on_cancel_route(self, _):
        if self.core.route is None:
            return pb.REJECT_REASON_NOT_IN_THIS_STATE
        self.route_exec.cancel()
        self.core.route_cancelled()
        return None

    def on_start_recording(self, _):
        why = self.core.may_start(pb.JOB_RECORD_ROUTE)
        if why:
            return why
        if self.node.good_fix() is None:
            return pb.REJECT_REASON_NOT_READY
        self.set_home()
        self.node.release_estop()     # the operator drives with the sticks; no mode change
        self.core.recording_started(Recorder(self.settings.record_spacing_m))
        return None

    def on_save_recording(self, cmd):
        rec = self.core.recording
        if rec is None:
            return pb.REJECT_REASON_NOT_IN_THIS_STATE
        if len(rec.points) < 2:
            return pb.REJECT_REASON_TOO_SHORT
        name = cmd.name.strip() or self.store.suggested_name()
        if self.store.name_taken(name):
            return pb.REJECT_REASON_NAME_TAKEN
        route = self.store.save(name, rec.points)
        self.log.info(f"route saved: '{route.name}' ({len(route.points)} points) as {route.id}")
        self.broadcast_routes()
        self.core.recording_ended()
        return None

    def on_save_route(self, cmd):
        """A mission planned on the map in the app (owner, 2026-10-12: missions are
        planned, not recorded). Moves nothing, so it is accepted in any state."""
        if len(cmd.waypoints) < 2:
            return pb.REJECT_REASON_TOO_SHORT
        points = [(w.latitude_deg, w.longitude_deg) for w in cmd.waypoints]
        xy = pic = None
        if cmd.map_name:
            # Planned on a placed map (Step D1): kept as points ON it, converted with the
            # map's own tie -- the same as placed its picture, so a waypoint sits exactly
            # where it was put relative to the walls.
            m = placed_map(self.node.maps, cmd.map_name)
            if m is not None:
                xy = [to_map(tuple(m.geo), lat, lon) for lat, lon in points]
            else:
                # A map not placed (Step D2): the points come as metres on its picture.
                m = map_info(self.node.maps, cmd.map_name)
                if m is None:
                    return pb.REJECT_REASON_UNKNOWN_MAP
                pic = [(w.map_x_m, w.map_y_m) for w in cmd.waypoints]
                xy = [from_picture(m, x, y) for x, y in pic]
                points = []
        name = cmd.name.strip() or self.store.suggested_name()
        if self.store.name_taken(name):
            return pb.REJECT_REASON_NAME_TAKEN
        route = self.store.save(name, points, cmd.map_name, xy, pic)
        self.log.info(f"mission saved: '{route.name}' ({len(cmd.waypoints)} waypoints) as {route.id}"
                      + (f" on map '{cmd.map_name}'" if cmd.map_name else ""))
        self.broadcast_routes()
        return None

    def on_delete_route(self, cmd):
        """Remove a saved mission (owner, 2026-10-15). Moves nothing, so it is accepted
        in any state -- except the mission being run or paused, which stays."""
        if self.store.get(cmd.route_id) is None:
            return pb.REJECT_REASON_UNKNOWN_ROUTE
        if self.core.route is not None and self.core.route_id == cmd.route_id:
            return pb.REJECT_REASON_NOT_IN_THIS_STATE
        name = self.store.get(cmd.route_id).name
        self.store.delete(cmd.route_id)
        self.log.info(f"mission deleted: '{name}' ({cmd.route_id})")
        self.broadcast_routes()
        return None

    def import_mission(self, cmd, sock):
        """ImportMission: the mission last sent to the autopilot (planned in QGC on
        another device), kept as a route. MAVROS reads it again first (the autopilot
        is only read, never written); the answer goes to `sock` when it has been
        read, or NO_MISSION after IMPORT_WAIT_S. Moves nothing: any state."""
        cid = cmd.command_id
        if cid and cid in self.importing:
            return                         # a resend while reading: one answer comes
        self.importing.add(cid)
        wanted = cmd.name.strip()
        answered = []

        def answer(why):
            if answered:
                return
            answered.append(True)
            self.importing.discard(cid)
            if why is None:
                res = pb.CommandResult(command_id=cid, outcome=pb.COMMAND_OUTCOME_ACCEPTED)
            else:
                res = pb.CommandResult(command_id=cid, outcome=pb.COMMAND_OUTCOME_REJECTED,
                                       reject_reason=why)
            self.log.info(f"import_mission from app {sock.request.remote_ip}: "
                          + ("accepted" if why is None else pb.RejectReason.Name(why)))
            if cid:
                self.remember(res)
            sock.send(command_result=res)

        def got(items):
            if answered:
                return
            if items is None:
                answer(pb.REJECT_REASON_NO_MISSION)
                return
            points = mission_waypoints(items)
            if not points:
                answer(pb.REJECT_REASON_NO_MISSION)
                return
            if len(points) < 2:
                answer(pb.REJECT_REASON_TOO_SHORT)
                return
            name = wanted or self.store.suggested_name()
            if self.store.name_taken(name):
                answer(pb.REJECT_REASON_NAME_TAKEN)
                return
            route = self.store.save(name, points)
            self.log.info(f"mission imported from the autopilot: '{route.name}' "
                          f"({len(points)} of {max(0, len(items) - 1)} items) as {route.id}")
            self.broadcast_routes()
            answer(None)

        self.node.read_mission(lambda items: self.loop.call_soon_threadsafe(got, items))
        self.loop.call_later(IMPORT_WAIT_S, answer, pb.REJECT_REASON_NO_MISSION)

    def on_discard_recording(self, _):
        if self.core.recording is None:
            return pb.REJECT_REASON_NOT_IN_THIS_STATE
        self.core.recording_ended()
        return None

    def on_route_failed(self, why):
        self.log.error(f"route: {why} -- paused, 'can't find a way'")
        self.core.route_failed()

    # ---------------- saved maps (Step C1) ----------------
    def map_list(self):
        return map_list_pb(self.node.maps)

    def broadcast_maps(self):
        ml = self.map_list()
        for s in self.apps():
            s.send(maps=ml)

    def map_command(self, kind, cmd, sock):
        """ShowMap / DeleteMap / StartMapping / SaveMap / DiscardMapping, done by the
        map manager (it owns the files and SLAM). Nothing moves: maps are files, and
        while mapping the operator drives with the sticks. No answer when the map
        manager is not running (the app says so)."""
        cid = cmd.command_id
        if cid and cid in self.map_calls:
            return                         # a resend while waiting: one answer comes
        name = getattr(getattr(cmd, kind), "name", "")
        answered = []

        def answer(result):
            if answered:
                return
            answered.append(True)
            self.map_calls.discard(cid)
            if result is None:
                self.log.error(f"{kind} '{name}' from app {sock.request.remote_ip}: no answer from the map manager")
                return
            ok, error = result
            if ok:
                res = pb.CommandResult(command_id=cid, outcome=pb.COMMAND_OUTCOME_ACCEPTED)
                if kind == "start_mapping":
                    # As for recording: the operator drives with the sticks, so a STOP
                    # latch is released; no mode change, nothing else moves.
                    self.node.release_estop()
                    self.core.mapping_started()
            else:
                why = MAP_ERRORS.get(error, pb.REJECT_REASON_NOT_SUPPORTED)
                res = pb.CommandResult(command_id=cid, outcome=pb.COMMAND_OUTCOME_REJECTED, reject_reason=why)
            self.log.info(f"{kind} '{name}' from app {sock.request.remote_ip}: "
                          + ("accepted" if ok else pb.RejectReason.Name(res.reject_reason)))
            if cid:
                self.remember(res)
            sock.send(command_result=res)

        def refuse(why):
            res = pb.CommandResult(command_id=cid, outcome=pb.COMMAND_OUTCOME_REJECTED, reject_reason=why)
            self.log.info(f"{kind} '{name}' from app {sock.request.remote_ip}: {pb.RejectReason.Name(why)}")
            if cid:
                self.remember(res)
            sock.send(command_result=res)

        if kind in BASE_COMMANDS:
            st = self.node.base_status
            if kind != "delete_base_site" and not (st is not None and st.heard):
                refuse(pb.REJECT_REASON_NO_BASE)
                return
            if kind in ("survey_base", "use_base_site", "save_base_site") and self.core.state in JOB_STATES:
                refuse(pb.REJECT_REASON_NOT_IN_THIS_STATE)
                return
        if kind in ("show_map", "delete_map"):
            known = self.node.maps is not None and any(m.name == name for m in self.node.maps.maps)
            if not known and not (kind == "show_map" and name == ""):
                answer((False, "unknown map"))
                return
        if kind == "start_mapping":
            why = self.core.may_start(pb.JOB_MAKE_MAP)   # another job, or the LiDAR not ready
            if why:
                res = pb.CommandResult(command_id=cid, outcome=pb.COMMAND_OUTCOME_REJECTED, reject_reason=why)
                self.log.info(f"start_mapping from app {sock.request.remote_ip}: {pb.RejectReason.Name(why)}")
                if cid:
                    self.remember(res)
                sock.send(command_result=res)
                return
        self.map_calls.add(cid)
        self.node.map_call(kind, name, lambda r: self.loop.call_soon_threadsafe(answer, r))
        self.loop.call_later(MAP_WAIT_S, answer, None)

    def broadcast_base_sites(self):
        bs = sites_pb(self.node.base_sites)
        for s in self.apps():
            s.send(base_sites=bs)

    def on_mapping_status(self):
        """The map manager says whether a map is being made (saved, discarded, restarted)."""
        st = self.node.mapping
        self.core.set_mapping(bool(st is not None and st.active))

    def route_list(self):
        return pb.RouteList(routes=[r.summary() for r in self.store.all()])

    def broadcast_routes(self):
        rl = self.route_list()
        for s in self.apps():
            s.send(routes=rl)

    def remember(self, res):
        self.results[res.command_id] = res
        while len(self.results) > 512:
            self.results.popitem(last=False)

    def on_estop_edge(self, on):
        if on:
            self.core.stop()
        else:
            self.core.release()

    def app_gone(self):
        """An app that completed the handshake disconnected: silent from now on."""
        self.core.link_tick([float("inf")])

    # ---------------- state ----------------
    def state_msg(self):
        st = pb.RoverState(operating_state=self.core.state)
        self.node.fill_state(st, self.core.checks)
        st.readiness.extend(self.core.readiness_msgs())
        st.active_alerts.extend(self.core.alerts.sorted())
        core = self.core
        here = self.node.good_fix()
        est = self.estimated_position()
        if est is not None:
            # GPS lost (Step D3): where odometry says the rover is since the last good fix --
            # not the autopilot's lat/lon, which is not tied to the Earth on its LiDAR source.
            st.position.CopyFrom(pb.Position(latitude_deg=est[0], longitude_deg=est[1],
                                             horizontal_accuracy_m=round(2.0 + 0.01 * self.lidar_m, 1),
                                             quality=pb.QUALITY_POOR, rtk=pb.RTK_STATUS_NONE,
                                             from_odometry=True))
            here = est
        on_map = self.route_exec is not None and self.route_exec.mode == "map"
        if core.route is not None and core.route_points and on_map:
            pose = self.node.slam_pose        # a map mission: on its map (Step D2)
            frac, left = progress(core.route_points, core.target, pose[1:3] if pose else None,
                                  core.route_total_m, dist=math.dist)
            st.route.route_id = core.route_id
            st.route.target_index = core.target + 1
            st.route.waypoint_count = len(core.route_points)
            st.route.fraction_done = frac
            st.route.distance_remaining_m = round(left, 2)
            if core.going_home:                # the way home on the map (Step D2+)
                m = map_info(self.node.maps, self.node.maps.localizing) if self.node.maps is not None else None
                for x, y in core.route_points[:200]:
                    if m is not None and m.placed and len(m.geo) == 5:
                        lat, lon = to_latlon(tuple(m.geo), x, y)
                        st.route.waypoints.add(latitude_deg=lat, longitude_deg=lon)
                    elif m is not None:
                        px, py = to_picture(m, x, y)
                        st.route.waypoints.add(map_x_m=round(px, 3), map_y_m=round(py, 3))
        elif core.route is not None and core.route_points:
            frac, left = progress(core.route_points, core.target, here, core.route_total_m)
            st.route.route_id = core.route_id
            st.route.target_index = core.target + 1
            st.route.waypoint_count = len(core.route_points)
            st.route.fraction_done = frac
            st.route.distance_remaining_m = round(left, 2)
            if core.going_home:                # the way home is not in the route list
                for lat, lon in core.route_points[:200]:
                    st.route.waypoints.add(latitude_deg=lat, longitude_deg=lon)
        if st.HasField("route") and core.route == "running":
            self.fill_plan(st.route)
        fs = follow_status(*self.node.latest_tracks())
        if fs is not None:
            st.follow.CopyFrom(fs)
        if self.trail is not None:
            st.home.latitude_deg, st.home.longitude_deg = self.trail.home
            st.home.quality = pb.QUALITY_GOOD
        elif self.map_home is not None:            # home on a saved map (Step D2+)
            name, hx, hy = self.map_home
            m = map_info(self.node.maps, name)
            if m is not None:
                px, py = to_picture(m, hx, hy)
                st.home_on_map.map_name, st.home_on_map.x_m, st.home_on_map.y_m = name, round(px, 3), round(py, 3)
                if m.placed and len(m.geo) == 5:   # on the Earth too
                    st.home.latitude_deg, st.home.longitude_deg = to_latlon(tuple(m.geo), hx, hy)
                    st.home.quality = pb.QUALITY_GOOD
        if core.recording is not None:
            for lat, lon in core.recording.points:
                st.recording.waypoints.add(latitude_deg=lat, longitude_deg=lon)
            st.recording.length_m = round(core.recording.length_m(), 2)
            st.recording.suggested_name = self.store.suggested_name()
        if self.base_on:
            st.base.CopyFrom(base_pb(self.node.base_status))
        ml = self.node.maps
        if self.maps_on and ml is not None and ml.localizing:
            mp = st.map_position
            mp.map_name = ml.localizing
            mp.located = self.node.slam_localized
            f = self.node.slam_finding
            mp.finding = (pb.MAP_FINDING_FOUND if mp.located
                          else f if f in (pb.MAP_FINDING_SEARCHING, pb.MAP_FINDING_NEED_HERE,
                                          pb.MAP_FINDING_NOT_FOUND_HERE, pb.MAP_FINDING_NEED_FRONT)
                          else pb.MAP_FINDING_SEARCHING)
            m, pose = map_info(ml, ml.localizing), self.node.slam_pose
            if mp.located and m is not None and pose is not None:
                mp.has_pose = True
                mp.x_m, mp.y_m = (round(v, 3) for v in to_picture(m, pose[1], pose[2]))
                mp.heading_deg = round(picture_heading(pose[3]), 1)
        mp = mapping_pb(self.node.mapping) if self.maps_on else None
        if mp is not None:
            st.mapping.CopyFrom(mp)
        return st

    # ---------------- GPS lost (Step D3) ----------------
    def fill_plan(self, rp):
        """The rover's plan for what it drives now (owner, 2026-10-23): Nav2's /plan from where the
        rover is through the points of this drive, as the app draws it -- lat/lon, or metres on a
        map that is not placed. Nothing when there is no fresh plan, or no way to place it."""
        ex, pts = self.route_exec, self.node.fresh_plan()
        if ex is None or not pts:
            return
        if ex.mode == "map":                   # a map mission (or the way home on a map)
            pair, ml = self.node.map_pose_and_odom(), self.node.maps
            m = map_info(ml, ml.localizing) if ml is not None and ml.localizing else None
            if pair is None or m is None:
                return
            t = map_to_odom(*pair)
            for x, y in pts:
                mx, my = odom_to_map(t, x, y)
                if m.placed and len(m.geo) == 5:
                    lat, lon = to_latlon(tuple(m.geo), mx, my)
                    rp.planned_path.add(latitude_deg=lat, longitude_deg=lon)
                else:
                    px, py = to_picture(m, mx, my)
                    rp.planned_path.add(map_x_m=round(px, 3), map_y_m=round(py, 3))
        else:                                  # GPS: now, or as it was when GPS was lost
            snap = self.lost_snap if ex.mode == "odom" else self.node.gps_snapshot()
            if snap is None:
                return
            for x, y in pts:
                lat, lon = odom_to_ll(x, y, snap)
                rp.planned_path.add(latitude_deg=lat, longitude_deg=lon)
        rp.planned_through_index = ex.batch[1]

    def estimated_position(self):
        """(lat, lon) from odometry while GPS is lost, else None."""
        if not self.gps.lost or self.lost_snap is None:
            return None
        odom = self.node.odom_now()
        return odom_to_ll(odom[0], odom[1], self.lost_snap) if odom is not None else None

    def parked(self):
        c = self.core
        return c.route != "running" and not c.following and c.recording is None and not c.mapping

    def switch_source(self, source_set, then):
        """Ask the Cube for EK3 source set 1 (GPS) or 2 (LiDAR); then(ok) on this loop."""
        if self.cube_source == source_set:
            then(True)
            return
        self.switching = True

        def done(ok):
            self.switching = False
            if ok:
                self.cube_source = source_set
            then(ok)
        self.node.set_ekf_source(source_set, lambda ok: self.loop.call_soon_threadsafe(done, ok))

    def on_gps_lost(self):
        before = self.gps.bad_since
        good = [snap for t, snap in self.snaps if before is None or t < before]
        self.lost_snap = good[-1] if good else (self.snaps[-1][1] if self.snaps else None)
        self.log.warn("GPS lost" + ("" if self.lost_snap else " -- and no good moment kept to go on from"))
        c, ex = self.core, self.route_exec
        if c.route != "running" or ex is None:
            return
        if ex.mode == "map":
            # A map mission stops and asks too (owner, 2026-10-23: "take a pause before carrying on":
            # Carry on or Come home), replacing "map missions carry on" (2026-10-19). The Cube goes
            # onto its LiDAR source at once, so either answer can drive.
            ex.pause()
            c.gps_ask_lidar_m = None
            c.route_paused(pb.STATE_REASON_GPS_LOST)
            self.switch_source(2, lambda ok: self.switched(2, ok))
        elif ex.mode == "gps":
            ex.pause()
            c.gps_ask_lidar_m = None
            c.route_paused(pb.STATE_REASON_GPS_LOST)

    def on_gps_back(self):
        self.log.warn("GPS back")
        self.lost_snap = None
        c = self.core
        if c.route == "paused" and c.pause_reason == pb.STATE_REASON_GPS_LOST:
            # The question is moot: an ordinary pause; carrying on drives on GPS again.
            c.pause_reason = pb.STATE_REASON_OPERATOR
            c.update(pb.STATE_REASON_OPERATOR)

    def watch_gps(self):
        now = time.monotonic()
        fix_type, acc, heading_ok = self.node.gps_sample()
        ev = self.gps.feed(now, fix_type, acc, heading_ok)
        if not self.gps.lost and self.gps.good_sample(fix_type, acc, heading_ok) \
                and (not self.snaps or now - self.snaps[-1][0] >= 1.0):
            snap = self.node.gps_snapshot()
            if snap is not None:
                self.snaps.append((now, snap))
        if ev == "lost":
            self.on_gps_lost()
        elif ev == "back":
            self.on_gps_back()
        odom = self.node.odom_now()
        c, ex = self.core, self.route_exec
        if self.trail is not None and not c.going_home and odom is not None:
            self.trail.add_odom(odom[0], odom[1])
        # Driven on odometry with no GPS: an allowance, then ask again.
        on_odometry = c.route == "running" and ex is not None and ex.mode == "odom" and self.gps.lost
        if on_odometry and odom is not None:
            if self.lidar_last is not None:
                self.lidar_m += math.dist(self.lidar_last, odom[:2])
            self.lidar_last = odom[:2]
            if self.lidar_m >= self.lidar_limit_m:
                self.log.warn(f"GPS still lost after {self.lidar_m:.0f} m on odometry: asking again")
                ex.pause()
                c.gps_ask_lidar_m = self.lidar_m
                c.route_paused(pb.STATE_REASON_GPS_LOST)
        else:
            self.lidar_last = None
        self.watch_indoor_switch()
        switching_on = bool(self.node.param("ekf_source_switching"))
        may_ask = switching_on and not self.switching and self.parked() and now >= self.switch_retry_at
        # GPS back and the rover parked: the Cube back on GPS.
        if self.cube_source == 2 and not self.gps.lost and may_ask:
            self.switch_source(1, lambda ok: self.switched(1, ok))
        # Step D2 (owner, 2026-10-21: "the gateway can do it whenever required"): no GPS and
        # parked -- indoors from the start, or GPS gone -- the Cube onto its LiDAR source, so a
        # map mission (or Follow) has a position to drive on.
        elif self.cube_source != 2 and self.gps.lost and may_ask:
            self.switch_source(2, lambda ok: self.switched(2, ok))
        self.drive_source = self.drive_source_check(switching_on)
        a, driving = c.alerts, c.route == "running"
        a.set(pb.ALERT_NO_GPS_ON_LIDAR, on_odometry, pb.SEVERITY_WARNING,
              {"metres_left": max(0, round(self.lidar_limit_m - self.lidar_m))})
        # Only when GPS went DURING the mission: one begun indoors never had it (Step D2).
        a.set(pb.ALERT_NO_GPS_ON_MAP, self.gps.lost and driving and ex is not None and ex.mode == "map"
              and self.map_mission_gps)
        base = self.node.base_status if self.base_on else None
        sending = base is not None and base.heard and base.mode == base.MODE_FIXED
        a.set(pb.ALERT_RTK_LOST, self.rtk_watch.feed(now, sending, fix_type) and not self.gps.lost)
        fcu, gnss, apart = self.node.disagreement()
        off = self.disagree.feed(now, not self.gps.lost and self.cube_source == 1, fcu, gnss, apart)
        c.set_position_disagrees(off, {"degrees": round(self.disagree.deg), "metres": round(self.disagree.m, 1)})

    def switched(self, source_set, ok):
        if ok:
            self.log.warn(f"FCU on EKF source set {source_set} ({'GPS' if source_set == 1 else 'LiDAR'})")
        else:
            self.switch_retry_at = time.monotonic() + 10.0
            self.log.error(f"FCU: EKF source set {source_set} refused; asking again in 10 s")

    def watch_indoor_switch(self):
        """The INDOOR switch (Step D2): when it moves, the Cube takes its source set."""
        pwm = self.node.rc_channel(self.indoor_switch) if self.indoor_switch > 0 else None
        if pwm is None or pwm <= 0:
            return
        pos = 1 if pwm < 1200 else 3 if pwm > 1800 else 2     # ArduPilot's aux switch thresholds
        if pos != self.switch_pos:
            first = self.switch_pos is None
            self.switch_pos = pos
            if not self.switching:
                self.cube_source = pos
            self.log.warn(f"INDOOR switch {'at' if first else 'moved to'} {('low', 'middle', 'high')[pos - 1]}: "
                          f"the Cube on EKF source set {pos}")

    def drive_source_check(self, switching_on):
        """CHECK_DRIVE_SOURCE (Step D2): the Cube has a position to drive a map mission on --
        GPS, or with none its LiDAR source -- and its estimator is steady while parked.
        Judged only while a map is in use: it blocks only map missions, and outdoors a
        heading dropout (GPS "lost" for a few seconds) must not make the rover NOT_READY.
        PENDING until the Cube is heard: nothing is known about its source before."""
        fc = self.core.checks.get(pb.CHECK_FLIGHT_CONTROLLER, (pb.CHECK_STATUS_PENDING, {}))[0]
        if fc == pb.CHECK_STATUS_PENDING:
            return pb.CHECK_STATUS_PENDING, {}
        ml = self.node.maps
        if not (self.maps_on and ml is not None and ml.localizing):
            return pb.CHECK_STATUS_PASS, {}
        if self.node.readiness.estimator_ok is False:
            return pb.CHECK_STATUS_FAIL, {}
        if not self.gps.lost or self.cube_source == 2:
            return pb.CHECK_STATUS_PASS, {}
        return pb.CHECK_STATUS_FAIL, ({} if switching_on else {"needs_switch": 1})

    def record_map_trail(self):
        """Where the rover goes on the map in use, while found on it (Step D2+): the way home
        indoors. Not while it is on its way home."""
        f = self.found_pose()
        if f is None or self.core.going_home:
            return
        if self.map_trail is None or self.map_trail.name != f[0]:
            on_it = self.map_home is not None and self.map_home[0] == f[0]
            self.map_trail = MapTrail(f[0], self.map_home[1:] if on_it else None)
        self.map_trail.add(f[1], f[2])

    def tick_state(self):
        checks = self.node.checks
        ds = self.drive_source
        if checks is not self.checks_in or (ds is not None and ds != self.core.checks.get(pb.CHECK_DRIVE_SOURCE)):
            # readiness replaces the dict once a second; the drive source is the gateway's own
            self.checks_in = checks
            if ds is not None:
                checks = dict(checks)
                checks[pb.CHECK_DRIVE_SOURCE] = ds
            self.core.set_checks(checks)
        ml = self.node.maps
        in_use = bool(self.maps_on and ml is not None and ml.localizing)
        self.core.set_on_map(in_use and self.node.slam_localized, in_use and self.gps.lost)
        self.core.set_following(self.node.following())
        here = None
        if self.core.state == pb.OPERATING_STATE_RECORDING:
            here = self.node.good_fix()
            if here is not None:
                self.core.recording.add(*here)
        # The trail for Return to Home: wherever the rover goes (jobs or sticks), not
        # while it is on its way home.
        if self.trail is not None and not self.core.going_home:
            here = here or self.node.good_fix()
            if here is not None:
                self.trail.add(*here)
        self.watch_gps()
        self.record_map_trail()
        self.core.link_tick([s.silence_ms() for s in self.apps()])
        for s in self.apps():
            s.send_state()

    def tick_heartbeat(self):
        for s in self.apps():
            s.send_heartbeat()


async def every(period_s, fn, log):
    """fn() at a fixed rate (no drift); an exception is logged, the timer goes on."""
    nxt = time.monotonic()
    while True:
        nxt += period_s
        try:
            fn()
        except Exception as e:  # noqa: BLE001
            log.error(f"{fn.__name__}: {e!r}")
        await asyncio.sleep(max(0.0, nxt - time.monotonic()))


def spin_quietly(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main():
    args = remove_ros_args(sys.argv)[1:]
    dry = "--stop-dry-run" in args
    # Our own Ctrl-C handling, as ui_server: rclpy's handler would shut ROS down first.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    node = RosIO()
    node.dry_run = node.dry_run or dry
    gw = Gateway(node, loop)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=spin_quietly, args=(executor,), daemon=True)
    spin.start()
    host, port = node.param("host"), int(node.param("port"))
    code = 0
    try:
        app = tornado.web.Application(
            [(r"/api", ApiSocket, {"gw": gw}), (r"/video", VideoSocket, {"gw": gw}),
             (r"/maps/(.+)\.png", MapPicture, {"gw": gw})],
            # keepalive only; safety rests on the heartbeat silence, not on this.
            # The timeout must exceed the interval (ui_server.py, 2026-09-29).
            websocket_ping_interval=2, websocket_ping_timeout=6,
            websocket_max_message_size=16384)
        app.listen(port, host)
        # systemd stops with SIGTERM; a background job ignores SIGINT
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, loop.stop)
        loop.create_task(every(gw.settings.state_ms / 1000.0, gw.tick_state, gw.log))
        loop.create_task(every(gw.settings.heartbeat_ms / 1000.0, gw.tick_heartbeat, gw.log))
        gw.log.info(f"{VERSION} on ws://{host}:{port}/api as '{gw.rover_id}'"
                    + (" -- STOP DRY RUN" if node.dry_run else ""))
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    except OSError as e:                   # port taken: say so and exit cleanly
        gw.log.fatal(f"cannot listen on {host}:{port}: {e}")
        code = 1
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin.join(timeout=2.0)
        node.readiness.close()             # the tf listener's own node and thread
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
