#!/usr/bin/env python3
"""ArUco-marker tracking for click-to-select following.

    /follow/marker_detections (person_detector: id + corners per frame)
    /d435/color/camera_info, /tf_static (camera mount), /scan_safety,
    /odometry/filtered
        -> /follow/marker_tracks (dagger_msgs/MarkerTracks): every marker, for the UI
    /follow/select (Int32): id_base + marker_id selects that marker; a person's id
                            or -1 clears the marker selection
        -> /follow/track_id      (Int32, published first)   } for the SELECTED
           /follow/target        (PoseStamped, base_footprint)} marker only, with
           /follow/target_source ("marker" | "marker+lidar")  } publish_target
        -> /follow/search_hint   (PoseStamped, base_footprint): where the selected
                                 marker was last seen, while it is not seen

The marker counterpart of person_tracker, on the same topics, so
follow_controller, the collision monitor and the browser page are shared. The
two never publish a target at the same time: a selection belongs to exactly one
of them (ids >= id_base are markers) and each clears its own when the other's
kind is selected. Logic and its rules: marker_tracking.py (no ROS;
tools/test_marker_tracking.py).

A marker can be on anything -- a car, another rover, a person's back. Its ID is
its identity: lost and seen again is the same target, and following resumes by
itself inside the controller's resume_window. Nothing is followed until the
operator selects a marker.

No TF listener (as person_tracker): base<-camera is static (/tf_static),
odom<-base is /odometry/filtered.
"""
import math
import time

import numpy as np
import rclpy
import tf2_ros
from dagger_msgs.msg import MarkerDetections, MarkerTrack, MarkerTracks
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Int32, String
from tf2_msgs.msg import TFMessage

from dagger_bringup.marker_tracking import MarkerTracker, quat_to_R
from dagger_bringup.person_tracking import scan_xy, to_base

NAN = float("nan")


class MarkerTrackerNode(Node):

    def __init__(self):
        super().__init__("marker_tracker")
        p = self.declare_parameter
        p("detections_topic", "/follow/marker_detections")
        p("camera_info_topic", "/d435/color/camera_info")
        p("scan_topic", "/scan_safety")
        p("odom_topic", "/odometry/filtered")
        p("tracks_topic", "/follow/marker_tracks")
        p("select_topic", "/follow/select")
        p("target_topic", "/follow/target")
        p("base_frame", "base_footprint")
        p("odom_frame", "odom")
        p("image_rotation_deg", 0)
        p("publish_target", True)
        # Printed side of the marker's black square, metres. 0 = not set: range
        # then comes from the LiDAR only (tools/marker_probe.py measures the size).
        p("marker_size", 0.0)
        # Markers that may be tracked, e.g. "7" or "3,7". "" = any id of the
        # dictionary. Listing yours is what keeps a stray decode from ever showing.
        # (dynamic typing: on a command line marker_ids:=7 arrives as an integer)
        p("marker_ids", "", ParameterDescriptor(dynamic_typing=True))
        p("id_base", 1000000)              # track id = id_base + marker id (people are below)
        p("min_hits", 3)
        p("min_side_px", 10.0)
        p("max_reproj_px", 2.0)
        p("max_tilt_deg", 75.0)
        p("min_range", 0.3)
        p("max_range", 15.0)
        p("lost_after", 0.5)
        p("reid_window", 30.0)             # = follow_controller resume_window
        p("resume_hits", 2)
        p("scan_max_age", 0.4)
        p("odom_timeout", 0.5)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        ids = [int(i) for i in str(g("marker_ids")).replace(";", ",").split(",") if i.strip()]
        self.tracker = MarkerTracker(
            id_base=int(g("id_base")), allowed_ids=ids or None, min_hits=int(g("min_hits")),
            min_side_px=float(g("min_side_px")), max_reproj_px=float(g("max_reproj_px")),
            max_tilt_deg=float(g("max_tilt_deg")), min_range=float(g("min_range")),
            max_range=float(g("max_range")), lost_after=float(g("lost_after")),
            reid_window=float(g("reid_window")), resume_hits=int(g("resume_hits")))
        self.size = float(g("marker_size"))
        self.base_frame, self.odom_frame = g("base_frame"), g("odom_frame")
        self.rot = int(g("image_rotation_deg")) % 360
        self.publish_target = bool(g("publish_target"))
        self.scan_max_age = float(g("scan_max_age"))
        self.odom_timeout = float(g("odom_timeout"))

        self.K, self.dist = None, None
        self.width, self.height = 640, 480
        self.cam = None                          # (t (3,), R (3, 3)) optical frame in base
        self.cam_frame = None
        self.scan, self.scan_time = None, 0.0
        self.odom = None                         # (tx, ty, yaw, monotonic receipt)
        self.last_state = None
        self.last_reject = None
        self.proc_ms = []
        self.n_targets = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.create_subscription(
            TFMessage, "/tf_static", self.on_tf_static,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.tracks_pub = self.create_publisher(MarkerTracks, g("tracks_topic"), 10)
        self.target_pub = self.create_publisher(PoseStamped, g("target_topic"), 10)
        self.id_pub = self.create_publisher(Int32, "/follow/track_id", 10)
        self.source_pub = self.create_publisher(String, "/follow/target_source", 10)
        self.hint_pub = self.create_publisher(PoseStamped, "/follow/search_hint", 10)
        self.create_subscription(CameraInfo, g("camera_info_topic"), self.on_info, be)
        self.create_subscription(LaserScan, g("scan_topic"), self.on_scan, be)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, be)
        self.create_subscription(MarkerDetections, g("detections_topic"), self.on_dets, 10)
        self.create_subscription(Int32, g("select_topic"), self.on_select, 10)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"marker_tracker: {g('detections_topic')} -> {g('tracks_topic')}; marker size "
            + (f"{self.size:.3f} m" if self.size > 0 else "NOT SET (range from the LiDAR only)")
            + f", ids {ids or 'any'}"
            + (f"; selected marker -> {g('target_topic')}" if self.publish_target else ""))

    def on_info(self, m):
        if self.K is None:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])            # fx, fy, cx, cy
            self.dist = list(m.d)
        if m.width and m.height:
            self.width, self.height = m.width, m.height

    def on_tf_static(self, m):
        for t in m.transforms:
            self.tf_buffer.set_transform_static(t, "tf_static")
        self.cam = None                           # re-resolve with the new transforms

    def on_scan(self, m):
        self.scan, self.scan_time = m, time.monotonic()

    def on_odom(self, m):
        if m.header.frame_id != self.odom_frame or m.child_frame_id != self.base_frame:
            self.get_logger().warn(
                f"odometry is {m.header.frame_id}->{m.child_frame_id}, expected "
                f"{self.odom_frame}->{self.base_frame}; ignoring it", throttle_duration_sec=10.0)
            return
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw, time.monotonic())

    def on_select(self, m):
        base = self.tracker.id_base
        if m.data < base:                         # -1 (clear) or a PERSON: not ours
            if self.tracker.selected_id >= 0:
                self.get_logger().info(
                    "marker selection cleared" + (" (a person was selected)" if m.data >= 0 else ""))
            self.tracker.clear()
            return
        ok, why = self.tracker.select(m.data)
        if ok:
            t = self.tracker.selected()
            self.get_logger().info(
                f"SELECTED marker {t.marker_id} at {t.range:.1f} m, "
                f"{math.degrees(t.bearing):+.0f} deg")
        else:
            self.get_logger().warn(f"select marker {m.data - base} ignored: {why}")

    def camera(self, frame):
        """Camera optical frame in base_footprint (static) -> (t, R) or None."""
        if self.cam is None or self.cam_frame != frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, frame, rclpy.time.Time())
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f"no TF {self.base_frame}<-{frame}: {str(e)[:80]}",
                                       throttle_duration_sec=5.0)
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            self.cam = (np.array([t.x, t.y, t.z]), quat_to_R((q.x, q.y, q.z, q.w)))
            self.cam_frame = frame
        return self.cam

    # ---------------- per frame ----------------
    def on_dets(self, m):
        t0 = time.perf_counter()
        now = time.monotonic()
        scan_ok = self.scan is not None and now - self.scan_time <= self.scan_max_age
        pts = (scan_xy(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                       self.scan.range_min, self.scan.range_max) if scan_ok else None)
        T = (self.odom[:3] if self.odom is not None and now - self.odom[3] <= self.odom_timeout
             else None)
        obs = []
        cam = self.camera(m.header.frame_id) if self.K is not None else None
        if cam is not None:
            for d in m.markers:
                obs.append(self.tracker.observe(d.marker_id, d.corners, self.K, self.dist,
                                                self.size, cam[0], cam[1], pts))
        tracks, target = self.tracker.update(obs, now, T)
        state = self.tracker.state(now)
        self.log_changes(state)
        if self.publish_target and self.tracker.selected_id >= 0:
            if target is not None:
                self.publish(m.header.stamp, target)
            elif T is not None:
                hint = self.tracker.search_hint(now)
                if hint is not None:
                    hx, hy = to_base(T, *hint)
                    hp = PoseStamped()
                    hp.header.stamp, hp.header.frame_id = m.header.stamp, self.base_frame
                    hp.pose.position.x, hp.pose.position.y = float(hx), float(hy)
                    hp.pose.orientation.w = 1.0
                    self.hint_pub.publish(hp)
        self.publish_tracks(m, tracks, state, now)
        self.proc_ms.append(1000 * (time.perf_counter() - t0))
        if m.markers and self.K is None:
            self.get_logger().warn("waiting for camera_info -- markers cannot be located yet",
                                   throttle_duration_sec=5.0)

    def publish(self, stamp, target):
        x, y, source = target
        b = math.atan2(y, x)
        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.base_frame
        out.pose.position.x, out.pose.position.y = float(x), float(y)
        out.pose.orientation.z, out.pose.orientation.w = math.sin(b / 2), math.cos(b / 2)
        # Id first: follow_controller disables on an id change, and must learn of a
        # different target before it receives that target's position.
        self.id_pub.publish(Int32(data=int(self.tracker.selected_id)))
        self.source_pub.publish(String(data=source))
        self.target_pub.publish(out)
        self.n_targets += 1

    def publish_tracks(self, m, tracks, state, now):
        out = MarkerTracks()
        out.header = m.header
        out.image_width = int(m.image_width or self.width)
        out.image_height = int(m.image_height or self.height)
        out.image_rotation_deg = self.rot
        out.odom_frame = self.odom_frame
        out.id_base = int(self.tracker.id_base)
        out.selected_id = int(self.tracker.selected_id)
        out.target_state = state
        for t in tracks:
            if t.corners is None:
                continue
            k = MarkerTrack()
            k.id, k.marker_id = int(t.id), int(t.marker_id)
            k.corners = [float(v) for v in t.corners]
            k.visible, k.confirmed = bool(t.visible), bool(t.confirmed and t.ever_ranged)
            k.age_s = float(now - t.born)
            k.unseen_s = float(now - t.t_seen) if not t.visible else 0.0
            k.bearing_deg = math.degrees(t.bearing) if t.bearing == t.bearing else NAN
            k.range_m = float(t.range)
            k.range_source = t.source
            k.heading_deg = math.degrees(t.heading) if t.heading == t.heading else NAN
            if t.pos is not None:
                k.position.x, k.position.y, k.position.z = float(t.pos[0]), float(t.pos[1]), 0.0
            else:
                k.position.x = k.position.y = k.position.z = NAN
            out.tracks.append(k)
        self.tracks_pub.publish(out)

    def log_changes(self, state):
        # One call site per severity (rclpy: "Logger severity cannot be changed
        # between calls" -- it crashed person_tracker once).
        if self.tracker.refound is not None:
            mid, gone = self.tracker.refound
            self.get_logger().warn(f"marker {mid} SEEN AGAIN after {gone:.1f} s -- same id, target resumes")
        if state != self.last_state:
            t = self.tracker.selected()
            mid = t.marker_id if t is not None else self.tracker.selected_id - self.tracker.id_base
            if state == "SEARCHING":
                self.get_logger().warn(
                    f"marker {mid} LOST (not seen for > {self.tracker.lost_after:.1f} s) -- "
                    f"searching: resumes if seen again within {self.tracker.reid_window:.0f} s")
            elif state == "LOST":
                self.get_logger().error(f"marker {mid} LOST -- no target until it is seen and "
                                        f"FOLLOW is pressed again")
            elif state == "TRACKING":
                self.get_logger().info(f"marker {mid}: camera has it")
            elif self.last_state is not None:
                self.get_logger().info("no marker selected")
            self.last_state = state
        r = self.tracker.last_reject
        if r and r != self.last_reject:
            self.get_logger().info(f"sighting not used -- {r}", throttle_duration_sec=2.0)
        self.last_reject = r

    def report(self):
        if self.proc_ms:
            p = sorted(self.proc_ms)
            s = self.tracker.stats
            shown = [f"{t.marker_id}" + ("" if t.confirmed else "?") for t in self.tracker.tracks.values()]
            sel = self.tracker.selected()
            self.get_logger().info(
                f"{len(p) / 10.0:.1f} Hz, markers {shown or 'none'}, selected "
                + (f"marker {sel.marker_id} {self.last_state}" if sel is not None else "none")
                + f"; {self.n_targets} targets, {s['rejected']} sightings rejected, "
                f"{s['gated']} outside the motion gate; update median {p[len(p) // 2]:.1f} ms")
        self.proc_ms.clear()
        self.n_targets = 0
        for k in self.tracker.stats:
            self.tracker.stats[k] = 0


def main():
    rclpy.init()
    node = MarkerTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
