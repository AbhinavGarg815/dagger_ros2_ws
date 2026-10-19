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
from .routes import (Recorder, RouteStore, Trail, distance_m, home_path, mission_waypoints,
                     path_length, progress)
from .base import base_pb, sites_pb
from .maps import MapPicture, map_list_pb, mapping_pb, placed_map, to_map
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
                      pb.CAPABILITY_IMPORT_MISSION, pb.CAPABILITY_COME_HOME, pb.CAPABILITY_FOLLOW)
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
                             + ((pb.CAPABILITY_MAPS,) if self.maps_on else ())
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
        if self.trail is None:
            here = self.node.good_fix()
            if here is not None:
                self.trail = Trail(here)
                self.log.info(f"home set at {here[0]:.7f}, {here[1]:.7f}")

    def on_route_done(self):
        if self.core.going_home and self.trail is not None:
            self.trail.reset()
            self.log.info("route: home reached")
        self.core.route_finished()

    # Phase 3 (docs/proposals/phase3-routes.md in DaggerGCS): each returns None when
    # accepted, else the reject reason. The rules are the core's; the ROS side is here.
    def on_start_route(self, cmd):
        why = self.core.may_start(pb.JOB_RUN_ROUTE)
        if why:
            return why
        route = self.store.get(cmd.route_id)
        if route is None:
            return pb.REJECT_REASON_UNKNOWN_ROUTE
        if route.map_name:
            # A map mission: its map must be the one the rover is finding itself on, and found.
            ml = self.node.maps
            if not (ml is not None and ml.localizing == route.map_name and self.node.slam_localized):
                return pb.REJECT_REASON_NOT_ON_MAP
        if not self.node.armed():
            return pb.REJECT_REASON_NOT_ARMED
        blocked = self.route_exec.precheck(route.points, route.xy)
        if blocked:
            return pb.REJECT_REASON_WAYPOINT_BLOCKED, blocked
        self.set_home()
        here = self.node.good_fix()
        total = (distance_m(here, route.points[0]) if here else 0.0) + path_length(route.points)
        self.node.release_estop()
        self.node.ensure_guided()
        self.core.route_started(route.id, route.points, total)
        self.route_exec.start(route.points, 0, route.xy)
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
        """Return to Home: back along the trail to where the session's first job started."""
        why = self.core.may_start(pb.JOB_COME_HOME)
        if why:
            return why
        if self.trail is None:
            return pb.REJECT_REASON_NO_HOME
        if not self.node.armed():
            return pb.REJECT_REASON_NOT_ARMED
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
        self.node.release_estop()
        self.node.ensure_guided()
        self.core.route_resumed()
        self.route_exec.resume()
        return None

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
        xy = None
        if cmd.map_name:
            # Planned on a placed map (Step D1): kept as points ON it, converted with the
            # map's own tie -- the same as placed its picture, so a waypoint sits exactly
            # where it was put relative to the walls.
            m = placed_map(self.node.maps, cmd.map_name)
            if m is None:
                return pb.REJECT_REASON_UNKNOWN_MAP
            xy = [to_map(tuple(m.geo), lat, lon) for lat, lon in points]
        name = cmd.name.strip() or self.store.suggested_name()
        if self.store.name_taken(name):
            return pb.REJECT_REASON_NAME_TAKEN
        route = self.store.save(name, points, cmd.map_name, xy)
        self.log.info(f"mission saved: '{route.name}' ({len(route.points)} waypoints) as {route.id}"
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
            if kind in ("survey_base", "use_base_site") and self.core.state in JOB_STATES:
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
        if core.route is not None and core.route_points:
            frac, left = progress(core.route_points, core.target, self.node.good_fix(), core.route_total_m)
            st.route.route_id = core.route_id
            st.route.target_index = core.target + 1
            st.route.waypoint_count = len(core.route_points)
            st.route.fraction_done = frac
            st.route.distance_remaining_m = round(left, 2)
            if core.going_home:                # the way home is not in the route list
                for lat, lon in core.route_points[:200]:
                    st.route.waypoints.add(latitude_deg=lat, longitude_deg=lon)
        fs = follow_status(*self.node.latest_tracks())
        if fs is not None:
            st.follow.CopyFrom(fs)
        if self.trail is not None:
            st.home.latitude_deg, st.home.longitude_deg = self.trail.home
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
            st.map_position.map_name = ml.localizing
            st.map_position.located = self.node.slam_localized
        mp = mapping_pb(self.node.mapping) if self.maps_on else None
        if mp is not None:
            st.mapping.CopyFrom(mp)
        return st

    def tick_state(self):
        checks = self.node.checks
        if checks is not self.checks_in:   # readiness replaces the dict once a second
            self.checks_in = checks
            self.core.set_checks(checks)
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
