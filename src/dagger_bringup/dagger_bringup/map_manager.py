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

USING A MAP (Step D1, map missions): while a placed map is shown under missions
(and no map is being made), slam.launch.py runs in SHADOW localization on it:
slam_toolbox in its own slam_map frame, no TF, nothing Nav2 uses;
slam_geo_init finds the start pose from GPS + a scan match and says
/slam/localized. MapList.localizing names that map. The gateway drives a map
mission's waypoints relative to /slam/pose.

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

The maps in src/dagger_bringup/maps (the engineer's, in git) are never touched.
The work is in map_store.py and map_tie.py (tested offline: tools/test_map_*.py).
"""
import math
import os
import queue
import signal
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from slam_toolbox.srv import SerializePoseGraph
from std_srvs.srv import Trigger

from dagger_msgs.msg import MapInfo, MapList, MappingStatus
from dagger_msgs.srv import MapName

from .map_store import PICTURES, MapStore, grid_to_pgm, picture_from_array, save_grid
from .map_tie import Tie, geo_yaml, to_latlon

LIVE = "~live"                 # the map being made, as the gateway names its picture
SERIALIZE_WAIT_S = 120.0       # a big pose graph takes a while to write
SLAM_STOP_WAIT_S = 15.0


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MapManager(Node):

    def __init__(self):
        super().__init__("map_manager")
        self.declare_parameter("map_dir", "~/dagger_maps")
        self.declare_parameter("rescan_s", 10.0)
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

        # The map being made: None, or the SLAM process and what has come in so far.
        self.slam = None               # subprocess.Popen of slam.launch.py
        self.subs = []
        self.tie = None
        self.grid = None               # the latest /slam/map
        self.live = None               # MapInfo of the map so far
        self.version = 0
        self.saving = False
        self.done = queue.Queue()      # the save worker's result, picked up on this thread
        self.loc = None                # subprocess.Popen of the shadow localization, or None
        self.loc_name = ""             # ... on this map

        self.store.scan()
        self.update_localization()
        self.publish()
        self.publish_status()
        self.create_timer(float(self.get_parameter("rescan_s").value), self.rescan)
        self.create_timer(0.5, self.after_save)
        self.get_logger().info(f"{len(self.store.maps)} maps in {self.store.folder}; "
                               f"shown: '{self.store.shown()}'")

    # ---------------- saved maps (C1) ----------------
    def rescan(self):
        if self.store.scan():
            self.get_logger().info(f"maps changed: {sorted(self.store.maps)}")
            self.update_localization()
            self.publish()

    def publish(self):
        out = MapList(shown=self.store.shown(), localizing=self.loc_name)
        for m in self.store.maps.values():
            out.maps.append(MapInfo(
                name=m.name, width_m=float(m.width_m), height_m=float(m.height_m), placed=m.placed,
                corner_lat=[c[0] for c in m.corners], corner_lon=[c[1] for c in m.corners],
                placed_error_m=float(m.placed_error_m), picture=m.picture,
                picture_width_px=m.width_px, picture_height_px=m.height_px,
                picture_version=m.version, created_ms=m.created_ms, geo=list(m.geo)))
        self.pub.publish(out)

    # ---------------- using a map: shadow localization (D1) ----------------
    def update_localization(self):
        """Localize on the shown map when it is placed and no map is being made; else not."""
        shown = self.store.shown()
        m = self.store.maps.get(shown)
        wanted = shown if (m is not None and m.placed and self.slam is None) else ""
        if wanted == self.loc_name:
            return
        self.stop_localization()
        if wanted:
            log = open(os.path.join(self.store.folder, ".localizing.log"), "w")
            self.loc = subprocess.Popen(
                ["ros2", "launch", "dagger_bringup", "slam.launch.py", "mode:=localization", "shadow:=true",
                 f"map:={self.store.path(wanted, '')}"],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            self.loc_name = wanted
            self.get_logger().info(f"finding the rover on '{wanted}' (shadow localization, pid {self.loc.pid})")

    def stop_localization(self):
        if self.loc is not None:
            stop_group(self.loc)
            self.get_logger().info(f"localization on '{self.loc_name}' stopped")
        self.loc, self.loc_name = None, ""

    def on_show(self, req, res):
        res.ok = self.store.show(req.name)
        res.error = "" if res.ok else "unknown map"
        if res.ok:
            self.get_logger().info(f"shown under missions: '{req.name}'")
            self.update_localization()
            self.publish()
        return res

    def on_delete(self, req, res):
        if req.name and req.name == self.loc_name:
            self.stop_localization()           # its files are about to go
        res.ok = self.store.delete(req.name)
        res.error = "" if res.ok else "unknown map"
        if res.ok:
            self.get_logger().info(f"map deleted: '{req.name}'")
        self.update_localization()
        self.publish()
        return res

    # ---------------- making a map (C2) ----------------
    def publish_status(self):
        st = MappingStatus(active=self.slam is not None, saving=self.saving,
                           suggested_name=self.store.suggested_name())
        if self.live is not None:
            st.live = self.live
        self.status_pub.publish(st)

    def on_start_mapping(self, _req, res):
        if self.slam is not None:
            res.success, res.message = False, "already mapping"
            return res
        self.stop_localization()               # one slam_toolbox at a time
        self.publish()
        log = open(os.path.join(self.store.folder, ".mapping.log"), "w")
        # Its own process group, so stopping it stops every node the launch started.
        self.slam = subprocess.Popen(["ros2", "launch", "dagger_bringup", "slam.launch.py"],
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.tie, self.grid, self.live, self.version, self.saving = Tie(), None, None, 0, False
        self.subs = [
            self.create_subscription(OccupancyGrid, "/slam/map", self.on_grid, self.latched),
            self.create_subscription(PoseWithCovarianceStamped, "/slam/pose", self.on_pose, 10),
            self.create_subscription(NavSatFix, "/mavros/global_position/global", self.on_fix,
                                     qos_profile_sensor_data),
            self.create_subscription(Imu, "/gps/heading_imu", self.on_heading, qos_profile_sensor_data),
        ]
        self.get_logger().info(f"mapping started (slam.launch.py, shadow; pid {self.slam.pid})")
        self.publish_status()
        res.success, res.message = True, ""
        return res

    def on_fix(self, m):
        if self.tie is not None and m.status.status >= 0:
            sigma = math.sqrt(max(m.position_covariance[0], m.position_covariance[4], 1e-4))
            self.tie.add_fix(stamp_s(m.header), m.latitude, m.longitude, sigma)

    def on_heading(self, m):
        if self.tie is not None:
            self.tie.add_heading(stamp_s(m.header), yaw_of(m.orientation))

    def on_pose(self, m):
        if self.tie is not None:
            p = m.pose.pose
            self.tie.add_pose(stamp_s(m.header), p.position.x, p.position.y, yaw_of(p.orientation))

    def on_grid(self, g):
        """A new /slam/map: redraw the picture of the map so far and tie it again."""
        if self.slam is None or self.saving:
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
                       picture_width_px=w, picture_height_px=h, picture_version=self.version)
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
        if self.slam is None or self.saving:
            res.ok, res.error = False, "not mapping"
            return res
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
                self.get_logger().info(f"map saved: '{name}'" + ("" if fit else " (not placed: no good GPS)"))
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
        if ok:
            self.stop_mapping()
            self.store.scan()
            self.publish()
        else:
            self.publish_status()          # still mapping: the operator may try again

    def on_discard_mapping(self, _req, res):
        if self.slam is None or self.saving:
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
        if self.slam is not None:
            stop_group(self.slam)
        self.slam, self.tie, self.grid, self.live = None, None, None, None
        pics = os.path.join(self.store.folder, PICTURES)
        for old in os.listdir(pics):
            if old.startswith(LIVE + "-"):
                os.remove(os.path.join(pics, old))
        self.publish_status()
        self.update_localization()             # back on the shown map, if any
        self.publish()


def stop_group(proc):
    """SIGINT a launch's whole process group; SIGKILL if it does not stop in time."""
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=SLAM_STOP_WAIT_S)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def main():
    rclpy.init()
    node = MapManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    for proc in (node.slam, node.loc):     # never leave SLAM running behind us
        if proc is not None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
