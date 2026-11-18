#!/usr/bin/env python3
"""The operator's maps, for the app (through the gateway). Steps C1 and C2.

    ros2 run dagger_bringup map_manager                          # ~/dagger_maps
    ros2 run dagger_bringup map_manager --ros-args -p map_dir:=/some/where

Out:  /maps/list     dagger_msgs/MapList, latched: every saved map (size, corners
                     when tied to the Earth, its PNG) and the one shown under missions
      /maps/mapping  dagger_msgs/MappingStatus, latched: the map being made
In:   /maps/show            dagger_msgs/srv/MapName  draw this map under missions ("" = none)
      /maps/delete          dagger_msgs/srv/MapName  remove a map's files for good
      /maps/start_mapping   std_srvs/srv/Trigger     start making a map
      /maps/save_map        dagger_msgs/srv/MapName  keep it ("" = the suggested name)
      /maps/discard_mapping std_srvs/srv/Trigger     throw it away

USING A MAP (Step D1, map missions): while a map is shown under missions (and no
map is being made), slam.launch.py runs in SHADOW localization on it: slam_toolbox
in its own slam_map frame, no TF, nothing Nav2 uses; slam_geo_init finds the start
pose -- from GPS + a scan match on a placed map, or (Step D2, indoor maps and when
GPS can't) around the spot the operator taps, "I am here" (/slam/here) -- and says
/slam/localized and /slam/finding. MapList.localizing names that map; MapInfo's
origin turns points on its picture into points on the map. The gateway drives a
map mission's waypoints relative to /slam/pose.

MAKING A MAP starts slam.launch.py in its SHADOW mapping mode as a child process:
slam_toolbox builds the map from /scan in its own slam_map frame and publishes
nothing Nav2, the GPS map layer or Follow use. Nothing moves: the operator
drives with the sticks. While mapping, the map's picture is redrawn from
/slam/map (every 5 s, slam_toolbox's map_update_interval) and tied to the
Earth live from /slam/pose, the GPS fixes and the dual-antenna heading
(map_tie.py: the same fit as tools/slam_georef.py). SAVE writes what the
localization launch reads -- <name>.posegraph/.data (serialize_map), .pgm/.yaml
(as map_saver_cli), _geo.yaml when tied -- then stops SLAM. It is answered at
once and done on a worker thread (the pose graph is tens of MB).

THE MAP OF THIS DRIVE (drive_map:=true; owner 2026-11-17, DaggerGCS docs/proposals/drive-map.md):
whenever no saved map is shown and none is being made, the same SHADOW mapping runs by itself, so
the rover always has a map of where it has been -- driven with the sticks or on a job. It is listed
as "~drive" (MapInfo name) with MapList.localizing = "~drive": the gateway uses it as the map in use
while GPS is not good enough to drive on (home, the way home and Go to a place on it). It is tied to
the Earth from the GPS seen on the way, as a map being made is. It is never saved: showing a saved
map or making one ends it (one slam_toolbox at a time); it starts again, new, afterwards.

NEVER ACROSS AN ODOMETRY LOSS OR JUMP (owner, 2026-11-18: after a dagger-nav restart the map of
this drive drew its walls twice): SLAM places each scan by odometry. When /odometry/filtered goes
silent (odom_lost_s: a restart or a crash) the running SLAM is stopped at once, before the reset
odometry can smear the map; when odometry is back, the map of this drive starts new and a saved map
is found again (GPS, or "I am here"). A jump (odom_jump_m / odom_jump_rad between two messages)
does the same, after a short hold. A map being made by the operator (Make map) is left alone and
only logged: the operator saves or discards it.

SAVING THE MAP OF THIS DRIVE: /maps/save_map while no map is being made keeps the map of this drive
as a saved map (the same files as Make map); the map of this drive goes on.

STARTING AND STOPPING SLAM is never done in a service call: a ros2 launch takes
seconds to stop, longer than the gateway waits (Stop showing went unanswered,
2026-10-16). The services answer at once with what was asked; the Launches
worker starts and stops the launches one at a time, in the order asked.

The maps in src/dagger_bringup/maps (the engineer's, in git) are never touched.
The work is in map_store.py and map_tie.py (tested offline: tools/test_map_*.py).
"""
import math
import os
import queue
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from mavros_msgs.msg import GPSRAW
from sensor_msgs.msg import Imu, NavSatFix
from slam_toolbox.srv import SerializePoseGraph
from std_srvs.srv import Trigger

from dagger_msgs.msg import MapInfo, MapList, MappingStatus
from dagger_msgs.srv import MapName

from .launches import Launches
from .map_store import PICTURES, MapStore, grid_to_pgm, picture_from_array, save_grid
from .map_tie import Tie, carry_fit, geo_yaml, to_latlon
from .odom_watch import OdomWatch

LIVE = "~live"                 # the map being made, as the gateway names its picture
DRIVE = "~drive"               # the map of this drive (drive_map), as it is listed
SERIALIZE_WAIT_S = 120.0       # a big pose graph takes a while to write


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MapManager(Node):

    def __init__(self):
        super().__init__("map_manager")
        self.declare_parameter("map_dir", "~/dagger_maps")
        self.declare_parameter("rescan_s", 10.0)
        # The map of this drive: OFF until its CPU cost is measured on the Jetson.
        self.declare_parameter("drive_map", False)
        self.drive_enabled = bool(self.get_parameter("drive_map").value)
        self.drive_on = False          # its shadow mapping runs
        self.drive_subs = []
        self.drive_info = None         # MapInfo of it, once SLAM has drawn something
        self.drive_grid = None         # its latest /slam/map, to save it
        self.saving_drive = False      # the save running is of the map of this drive
        # Odometry watch: SLAM never draws across a loss or a jump of it.
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("odom_lost_s", 2.0)
        self.declare_parameter("odom_jump_m", 1.5)       # between two messages under 0.5 s apart
        self.declare_parameter("odom_jump_rad", 0.5)     # between two messages under 0.2 s apart
        self.odom = OdomWatch(float(self.get_parameter("odom_lost_s").value),
                              float(self.get_parameter("odom_jump_m").value),
                              float(self.get_parameter("odom_jump_rad").value))
        self.slam_hold_until = 0.0     # monotonic s: no SLAM started before this (after a jump)
        # The Earth tie carried across a restart of the map of this drive (indoors there is no GPS
        # to tie the new one): (old fit, the rover's pose on the old map) when it stopped, then the
        # new map's fit once its first pose is known.
        self.slam_pose_last = None     # (x, y, yaw) of the latest /slam/pose
        self.carry = None
        self.carried_fit = None
        self.store = MapStore(self.get_parameter("map_dir").value)
        self.latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                  history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(MapList, "/maps/list", self.latched)
        self.status_pub = self.create_publisher(MappingStatus, "/maps/mapping", self.latched)
        self.create_service(MapName, "/maps/show", self.on_show)
        self.create_service(MapName, "/maps/delete", self.on_delete)
        self.create_service(Trigger, "/maps/start_mapping", self.on_start_mapping)
        self.create_service(MapName, "/maps/save_map", self.on_save_map)
        self.create_service(Trigger, "/maps/discard_mapping", self.on_discard_mapping)
        self.serialize_cli = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")

        # The map being made: whether one is, and what has come in so far.
        self.mapping_on = False
        self.subs = []
        self.tie = None
        self.grid = None               # the latest /slam/map
        self.live = None               # MapInfo of the map so far
        self.version = 0
        self.saving = False
        self.done = queue.Queue()      # the save worker's result, picked up on this thread
        self.loc_name = ""             # the map asked to localize on ("" = none)
        self.launches = Launches(self.store.folder, self.get_logger())

        self.store.scan()
        self.update_localization()
        self.publish()
        self.publish_status()
        self.create_timer(float(self.get_parameter("rescan_s").value), self.rescan)
        self.create_timer(0.5, self.after_save)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom,
                                 qos_profile_sensor_data)
        self.create_timer(0.5, self.watch_odom)
        self.get_logger().info(f"{len(self.store.maps)} maps in {self.store.folder}; "
                               f"shown: '{self.store.shown()}'")

    # ---------------- saved maps (C1) ----------------
    def rescan(self):
        if self.store.scan():
            self.get_logger().info(f"maps changed: {sorted(self.store.maps)}")
            self.update_localization()
            self.publish()

    def publish(self):
        localizing = self.loc_name or (DRIVE if self.drive_on else "")
        out = MapList(shown=self.store.shown(), localizing=localizing)
        if self.drive_on and self.drive_info is not None:
            out.maps.append(self.drive_info)
        for m in self.store.maps.values():
            out.maps.append(MapInfo(
                name=m.name, width_m=float(m.width_m), height_m=float(m.height_m), placed=m.placed,
                corner_lat=[c[0] for c in m.corners], corner_lon=[c[1] for c in m.corners],
                placed_error_m=float(m.placed_error_m), picture=m.picture,
                picture_width_px=m.width_px, picture_height_px=m.height_px,
                picture_version=m.version, created_ms=m.created_ms, geo=list(m.geo),
                origin_x=float(m.origin[0]), origin_y=float(m.origin[1])))
        self.pub.publish(out)

    # ---------------- using a map: shadow localization (D1) ----------------
    def update_localization(self):
        """Localize on the shown map, placed or not (Step D2), while no map is being made; with
        none shown, make the map of this drive (drive_map). One slam_toolbox at a time: the one
        going stops first (the worker keeps the order)."""
        shown = self.store.shown()
        m = self.store.maps.get(shown)
        held = self.odom.lost or time.monotonic() < self.slam_hold_until
        wanted = shown if (m is not None and not self.mapping_on and not held) else ""
        drive = self.drive_enabled and not self.mapping_on and not wanted and not held
        changed = False
        if self.drive_on and not drive:
            self.drive_stop()
            changed = True
        if wanted != self.loc_name:
            self.loc_name = wanted
            self.launches.localize(wanted)     # done on the worker; answered already
            changed = True
        if drive and not self.drive_on:
            self.drive_start()
            changed = True
        return changed

    # ---------------- odometry watch ----------------
    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        said = self.odom.step(time.monotonic(), p.x, p.y, yaw_of(q))
        if said == "back":
            self.get_logger().warn("odometry is back: SLAM starts again (a new map of this drive, "
                                   "or finding the rover on the shown map again)")
            self.slam_hold_until = time.monotonic() + 1.0     # let the new odometry settle first
        elif said == "jump":
            self.slam_off("odometry jumped", hold_s=1.0)

    def watch_odom(self):
        """Every 0.5 s: odometry silent -> SLAM off; a hold ended -> SLAM on again."""
        now = time.monotonic()
        if self.odom.check(now) == "lost":
            self.slam_off(f"odometry silent for more than {self.odom.lost_s:g} s (restart or crash)")
        elif not self.odom.lost and self.slam_hold_until and now >= self.slam_hold_until:
            self.slam_hold_until = 0.0
            if self.update_localization():
                self.publish()

    def slam_off(self, why, hold_s=0.0):
        """Stop the SLAM that odometry places (the map of this drive, or finding the rover on the
        shown map) before a bad odometry is drawn into it; it starts again after the hold."""
        if hold_s:
            self.slam_hold_until = time.monotonic() + hold_s
        if self.mapping_on:
            self.get_logger().error(f"{why}: the map being made may be wrong from here -- save or "
                                    "discard it (left running)")
            return
        if not (self.drive_on or self.loc_name):
            return
        self.get_logger().warn(f"{why}: SLAM stopped so no wrong place is drawn into the map")
        if self.update_localization():
            self.publish()
        self.carry_ok = True               # the next map of this drive may take the old tie

    # ---------------- the map of this drive ----------------
    def drive_start(self):
        if not getattr(self, "carry_ok", False):
            self.carry = None                  # not after an odometry restart: nothing to carry
        self.carry_ok = False
        self.drive_on, self.drive_info, self.version = True, None, 0
        self.tie = Tie()
        self.launches.map(True)
        self.drive_subs = [
            self.create_subscription(OccupancyGrid, "/slam/map", self.on_drive_grid, self.latched),
            self.create_subscription(PoseWithCovarianceStamped, "/slam/pose", self.on_pose, 10),
            self.create_subscription(NavSatFix, "/mavros/global_position/global", self.on_fix,
                                     qos_profile_sensor_data),
            self.create_subscription(Imu, "/gps/heading_imu", self.on_heading, qos_profile_sensor_data),
            self.create_subscription(GPSRAW, "/mavros/gpsstatus/gps1/raw", self.on_raw, qos_profile_sensor_data),
        ]
        self.get_logger().info("the map of this drive: mapping (no saved map shown)")

    def drive_stop(self):
        # Keep where the rover was on the Earth, to tie the next map of this drive (carry_fit).
        fit = self.tie.fit() if self.tie is not None else None
        fit = fit or self.carried_fit
        self.carry = (fit, self.slam_pose_last) if fit is not None and self.slam_pose_last is not None else None
        self.carried_fit, self.slam_pose_last = None, None
        for sub in self.drive_subs:
            self.destroy_subscription(sub)
        self.drive_subs = []
        self.launches.map(False)
        self.drive_on, self.drive_info, self.drive_grid, self.tie = False, None, None, None
        self.remove_pictures(DRIVE)
        self.get_logger().info("the map of this drive: stopped (a saved map shown, or a map being made)")

    def remove_pictures(self, prefix, keep=None):
        pics = os.path.join(self.store.folder, PICTURES)
        for old in os.listdir(pics):
            if old.startswith(prefix + "-") and old != keep:
                os.remove(os.path.join(pics, old))

    def on_drive_grid(self, g):
        """A new /slam/map while making the map of this drive: its picture, tied when it can be."""
        if not self.drive_on:
            return
        self.version += 1
        path = os.path.join(self.store.folder, PICTURES, f"{DRIVE}-{self.version}.png")
        picture_from_array(grid_to_pgm(g), path)
        self.remove_pictures(DRIVE, keep=os.path.basename(path))
        self.drive_grid = g
        fit = self.tie.fit() if self.tie is not None else None
        # GPS on the way ties it; until then, the tie carried over a restart (if any)
        self.drive_info = self.info_of(DRIVE, g, path, fit or self.carried_fit)
        self.publish()

    def info_of(self, name, g, path, fit):
        """MapInfo of a map being drawn from /slam/map: its picture, and its corners and tie once fitted."""
        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        info = MapInfo(name=name, width_m=float(w * res), height_m=float(h * res), picture=path,
                       picture_width_px=w, picture_height_px=h, picture_version=self.version,
                       origin_x=float(ox), origin_y=float(oy))
        if fit is not None:
            top, right = oy + h * res, ox + w * res
            corners = [to_latlon(fit, x, y) for x, y in ((ox, top), (right, top), (right, oy), (ox, oy))]
            info.placed = True
            info.corner_lat = [c[0] for c in corners]
            info.corner_lon = [c[1] for c in corners]
            info.placed_error_m = float(fit["rms"])
            info.geo = [float(fit["lat0"]), float(fit["lon0"]), float(fit["rot"]), float(fit["tx"]), float(fit["ty"])]
        return info

    def on_show(self, req, res):
        res.ok = self.store.show(req.name)
        res.error = "" if res.ok else "unknown map"
        if res.ok:
            self.get_logger().info(f"shown under missions: '{req.name}'")
            self.update_localization()
            self.publish()
        return res

    def on_delete(self, req, res):
        res.ok = self.store.delete(req.name)
        res.error = "" if res.ok else "unknown map"
        if res.ok:
            self.get_logger().info(f"map deleted: '{req.name}'")
        self.update_localization()
        self.publish()
        return res

    # ---------------- making a map (C2) ----------------
    def publish_status(self):
        st = MappingStatus(active=self.mapping_on, saving=self.saving,
                           suggested_name=self.store.suggested_name())
        if self.live is not None:
            st.live = self.live
        self.status_pub.publish(st)

    def on_start_mapping(self, _req, res):
        if self.mapping_on:
            res.success, res.message = False, "already mapping"
            return res
        self.mapping_on = True
        self.update_localization()             # one slam_toolbox at a time: localization / the drive's map off first
        self.launches.map(True)
        self.publish()
        self.tie, self.grid, self.live, self.version, self.saving = Tie(), None, None, 0, False
        self.subs = [
            self.create_subscription(OccupancyGrid, "/slam/map", self.on_grid, self.latched),
            self.create_subscription(PoseWithCovarianceStamped, "/slam/pose", self.on_pose, 10),
            self.create_subscription(NavSatFix, "/mavros/global_position/global", self.on_fix,
                                     qos_profile_sensor_data),
            self.create_subscription(Imu, "/gps/heading_imu", self.on_heading, qos_profile_sensor_data),
            # the receiver's own accuracy for each fix (the tie, stricter 2026-10-23)
            self.create_subscription(GPSRAW, "/mavros/gpsstatus/gps1/raw", self.on_raw, qos_profile_sensor_data),
        ]
        self.publish_status()
        res.success, res.message = True, ""
        return res

    def on_raw(self, m):
        self.raw = (time.monotonic(), m.fix_type, m.h_acc / 1000.0 if m.h_acc > 0 else None)

    def on_fix(self, m):
        """A fix for the tie, with the RECEIVER's accuracy (gps1/raw): the autopilot's own
        covariance said a few metres whether RTK fixed or indoors (Map 4, 2026-10-23)."""
        raw = getattr(self, "raw", None)
        if self.tie is None or m.status.status < 0 or raw is None or time.monotonic() - raw[0] > 1.0:
            return
        _, fix_type, h_acc = raw
        if fix_type < 3 or h_acc is None:
            return
        self.tie.add_fix(stamp_s(m.header), m.latitude, m.longitude, max(h_acc, 0.01))

    def on_heading(self, m):
        if self.tie is not None:
            self.tie.add_heading(stamp_s(m.header), yaw_of(m.orientation))

    def on_pose(self, m):
        p = m.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.drive_on and self.carry is not None and self.carried_fit is None:
            # the new map's first pose: the same spot on the Earth as the last one on the old map
            self.carried_fit = carry_fit(self.carry[0], self.carry[1], pose)
            self.get_logger().info(f"the map of this drive: its Earth tie carried over the restart "
                                   f"(about {self.carried_fit['rms']:.1f} m)")
        self.slam_pose_last = pose
        if self.tie is not None:
            self.tie.add_pose(stamp_s(m.header), p.position.x, p.position.y, yaw_of(p.orientation))

    def on_grid(self, g):
        """A new /slam/map: redraw the picture of the map so far and tie it again."""
        if not self.mapping_on or self.saving:
            return
        self.grid = g
        self.version += 1
        pics = os.path.join(self.store.folder, PICTURES)
        path = os.path.join(pics, f"{LIVE}-{self.version}.png")
        picture_from_array(grid_to_pgm(g), path)
        for old in os.listdir(pics):
            if old.startswith(LIVE + "-") and old != os.path.basename(path):
                os.remove(os.path.join(pics, old))
        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        fit = self.tie.fit()
        info = MapInfo(name="", width_m=float(w * res), height_m=float(h * res), picture=path,
                       picture_width_px=w, picture_height_px=h, picture_version=self.version,
                       origin_x=float(ox), origin_y=float(oy))
        if fit is not None:
            top, right = oy + h * res, ox + w * res
            corners = [to_latlon(fit, x, y) for x, y in ((ox, top), (right, top), (right, oy), (ox, oy))]
            info.placed = True
            info.corner_lat = [c[0] for c in corners]
            info.corner_lon = [c[1] for c in corners]
            info.placed_error_m = float(fit["rms"])
        self.live = info
        self.publish_status()

    def on_save_map(self, req, res):
        if self.saving:
            res.ok, res.error = False, "not mapping"
            return res
        if not self.mapping_on:
            return self.save_drive(req, res)
        if self.grid is None:
            res.ok, res.error = False, "nothing mapped"
            return res
        name = self.store.safe_name(req.name) or self.store.suggested_name()
        if self.store.name_taken(name):
            res.ok, res.error = False, "name taken"
            return res
        self.saving = True
        self.publish_status()
        grid, fit = self.grid, self.tie.fit()
        threading.Thread(target=self.save_worker, args=(name, grid, fit), daemon=True).start()
        res.ok, res.error = True, ""
        return res

    def save_drive(self, req, res):
        """Keep the map of this drive as a saved map (owner, 2026-11-18). It goes on."""
        if not self.drive_on:
            res.ok, res.error = False, "not mapping"
            return res
        if self.drive_grid is None:
            res.ok, res.error = False, "nothing mapped"
            return res
        name = self.store.safe_name(req.name) or self.store.suggested_name()
        if self.store.name_taken(name):
            res.ok, res.error = False, "name taken"
            return res
        self.saving, self.saving_drive = True, True
        grid, fit = self.drive_grid, self.tie.fit() if self.tie is not None else None
        threading.Thread(target=self.save_worker, args=(name, grid, fit), daemon=True).start()
        self.get_logger().info(f"saving the map of this drive as '{name}'")
        res.ok, res.error = True, ""
        return res

    def save_worker(self, name, grid, fit):
        """Off the executor: wait for the pose graph, write the files, stop SLAM."""
        ok = True
        try:
            base = os.path.join(self.store.folder, name)
            if self.serialize_cli.wait_for_service(timeout_sec=5.0):
                fut = self.serialize_cli.call_async(SerializePoseGraph.Request(filename=base))
                t0 = time.monotonic()
                while not fut.done() and time.monotonic() - t0 < SERIALIZE_WAIT_S:
                    time.sleep(0.1)
                r = fut.result() if fut.done() else None
                if r is None or r.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                    self.get_logger().error(f"save '{name}': serialize_map failed ({r})")
                    ok = False
            else:
                self.get_logger().error(f"save '{name}': /slam_toolbox/serialize_map not available")
                ok = False
            if ok:
                save_grid(self.store.folder, name, grid)
                if fit is not None:
                    with open(base + "_geo.yaml", "w") as f:
                        f.write(geo_yaml(fit, f"saved {time.strftime('%Y-%m-%d %H:%M')}"))
                self.get_logger().info(f"map saved: '{name}'" + (
                    f" (placed: turn {math.degrees(fit['rot']):+.2f} deg, rms {fit['rms']:.2f} m over "
                    f"{fit['kept']} pairs)" if fit else " (not placed: no good GPS)"))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"save '{name}': {e!r}")
            ok = False
        self.done.put(ok)

    def after_save(self):
        """On this thread: the worker's result. Saved: stop SLAM, list it. Not: keep mapping."""
        try:
            ok = self.done.get_nowait()
        except queue.Empty:
            return
        self.saving = False
        if self.saving_drive:                  # the map of this drive: listed, and it goes on
            self.saving_drive = False
            self.store.scan()
            self.publish()
            return
        if ok:
            self.stop_mapping()
            self.store.scan()
            self.publish()
        else:
            self.publish_status()          # still mapping: the operator may try again

    def on_discard_mapping(self, _req, res):
        if not self.mapping_on or self.saving:
            res.success, res.message = False, "not mapping"
            return res
        self.stop_mapping()
        self.get_logger().info("mapping discarded")
        res.success, res.message = True, ""
        return res

    def stop_mapping(self):
        for s in self.subs:
            self.destroy_subscription(s)
        self.subs = []
        self.launches.map(False)               # stopped on the worker; nothing waits here
        self.mapping_on = False
        self.tie, self.grid, self.live = None, None, None
        pics = os.path.join(self.store.folder, PICTURES)
        for old in os.listdir(pics):
            if old.startswith(LIVE + "-"):
                os.remove(os.path.join(pics, old))
        self.publish_status()
        self.update_localization()             # back on the shown map, if any
        self.publish()


def main():
    rclpy.init()
    node = MapManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.launches.kill_all()           # never leave SLAM running behind us
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
