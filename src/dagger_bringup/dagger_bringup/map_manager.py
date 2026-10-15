#!/usr/bin/env python3
"""The operator's saved maps, for the app (through the gateway). Step C1.

    ros2 run dagger_bringup map_manager                          # ~/dagger_maps
    ros2 run dagger_bringup map_manager --ros-args -p map_dir:=/some/where

Out:  /maps/list    dagger_msgs/MapList, latched: every map (size, corners when
                    tied to the Earth, its PNG) and the one shown under missions
In:   /maps/show    dagger_msgs/srv/MapName  draw this map under missions ("" = none)
      /maps/delete  dagger_msgs/srv/MapName  remove a map's files for good

Reads files only: no SLAM is started, nothing on the rover moves. The folder
is read again every `rescan_s`, so a map copied in by hand appears by itself.
The maps in src/dagger_bringup/maps (the engineer's, in git) are never touched.
The work is in map_store.py (tested offline: tools/test_map_store.py).
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from dagger_msgs.msg import MapInfo, MapList
from dagger_msgs.srv import MapName

from .map_store import MapStore


class MapManager(Node):

    def __init__(self):
        super().__init__("map_manager")
        self.declare_parameter("map_dir", "~/dagger_maps")
        self.declare_parameter("rescan_s", 10.0)
        self.store = MapStore(self.get_parameter("map_dir").value)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(MapList, "/maps/list", latched)
        self.create_service(MapName, "/maps/show", self.on_show)
        self.create_service(MapName, "/maps/delete", self.on_delete)
        self.store.scan()
        self.publish()
        self.create_timer(float(self.get_parameter("rescan_s").value), self.rescan)
        self.get_logger().info(f"{len(self.store.maps)} maps in {self.store.folder}; "
                               f"shown: '{self.store.shown()}'")

    def rescan(self):
        if self.store.scan():
            self.get_logger().info(f"maps changed: {sorted(self.store.maps)}")
            self.publish()

    def publish(self):
        out = MapList(shown=self.store.shown())
        for m in self.store.maps.values():
            out.maps.append(MapInfo(
                name=m.name, width_m=float(m.width_m), height_m=float(m.height_m), placed=m.placed,
                corner_lat=[c[0] for c in m.corners], corner_lon=[c[1] for c in m.corners],
                placed_error_m=float(m.placed_error_m), picture=m.picture,
                picture_width_px=m.width_px, picture_height_px=m.height_px,
                picture_version=m.version, created_ms=m.created_ms))
        self.pub.publish(out)

    def on_show(self, req, res):
        res.ok = self.store.show(req.name)
        res.error = "" if res.ok else "unknown map"
        if res.ok:
            self.get_logger().info(f"shown under missions: '{req.name}'")
            self.publish()
        return res

    def on_delete(self, req, res):
        res.ok = self.store.delete(req.name)
        res.error = "" if res.ok else "unknown map"
        if res.ok:
            self.get_logger().info(f"map deleted: '{req.name}'")
            self.publish()
        return res


def main():
    rclpy.init()
    node = MapManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
