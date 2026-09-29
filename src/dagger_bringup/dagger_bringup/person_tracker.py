#!/usr/bin/env python3
"""Every detected person -> a stable ID and position; the operator's click -> the target.

    /follow/person_detections (person_detector: boxes + re-ID signatures)
    + /d435/color/camera_info + /scan_safety + /odometry/filtered + /tf_static
        -> /follow/tracks        (dagger_msgs/PersonTracks, one per camera frame, for the UI)
    /follow/select (Int32: track id, -1 = clear, from ui_server)
        -> /follow/target        (PoseStamped, base_footprint)   } only for the SELECTED
           /follow/track_id      (Int32, published first)         } person, and only
           /follow/target_source ("camera"|"lidar"|"near")        } with publish_target
        -> /follow/search_hint   (PoseStamped, base_footprint): where the selected
           person probably is while unseen (coasting / lost inside the re-ID window)

Replaces target_selector when follow.launch runs with target_source:=tracker.
Same sensors, same geometry, same parameter values: the camera gives each
person's bearing (box centre + intrinsics + the STATIC camera transform, from
the camera's real origin), the LiDAR the range (nearest cluster in the box's
angular window) -- the D435 depth is useless in sunlight and is not enabled.
Position tracking in odom, association and the LiDAR hold live in
person_tracking.py (no ROS; tools/test_person_tracking.py).

What target_selector did on its own -- pick "whoever is ahead" -- is NOT done
here: nothing is followed until the operator selects someone, and the tracker
never moves the selection to another person. If the selected person is lost,
the target stops and the page shows LOST until the operator clicks again.
Selecting a different person changes /follow/track_id, which makes
follow_controller disable itself: following someone new needs a new FOLLOW.

No TF listener (in Python it decodes all of /tf, ~16-25 % CPU): base<-camera is
static (/tf_static), odom<-base is /odometry/filtered.
"""
import math
import time

import numpy as np
import rclpy
import tf2_ros
from dagger_msgs.msg import PersonTrack, PersonTracks
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Int32, String
from tf2_msgs.msg import TFMessage
from dagger_msgs.msg import PersonDetections

from dagger_bringup.person_tracking import (Detection, Tracker, person_sized, range_at,
                                            scan_xy, to_base)

NAN = float("nan")


def quat_rotate(q, v):
    """Rotate vector v by quaternion q (geometry_msgs Quaternion)."""
    u = np.array([q.x, q.y, q.z])
    s = q.w
    return 2.0 * np.dot(u, v) * u + (s * s - np.dot(u, u)) * v + 2.0 * s * np.cross(u, v)


class PersonTracker(Node):

    def __init__(self):
        super().__init__("person_tracker")
        p = self.declare_parameter
        p("detections_topic", "/follow/person_detections")
        p("camera_info_topic", "/d435/color/camera_info")
        p("scan_topic", "/scan_safety")
        p("odom_topic", "/odometry/filtered")
        p("tracks_topic", "/follow/tracks")
        p("select_topic", "/follow/select")
        p("target_topic", "/follow/target")
        p("base_frame", "base_footprint")
        p("odom_frame", "odom")
        p("image_rotation_deg", 0)
        # Publish /follow/target for the selected person. False when the old
        # target_selector is the target source (only one node may publish it).
        p("publish_target", True)
        # --- same values as target_selector (validated on the rover 09-25/26) ---
        p("min_range", 1.0)
        p("max_range", 15.0)
        p("angular_window_scale", 1.2)
        p("min_window_deg", 2.0)
        p("max_window_deg", 25.0)
        p("cluster_depth", 0.6)
        p("min_cluster_points", 2)
        p("scan_max_age", 0.4)
        p("odom_timeout", 0.5)
        p("lidar_only_timeout", 5.0)
        p("track_gate", 0.6)
        p("max_person_speed", 1.8)
        p("max_cluster_width", 0.9)
        p("min_track_points", 3)
        # --- tracker ---
        p("iou_min", 0.2)
        p("new_track_score", 0.5)
        p("max_missed", 20)                # camera frames a confirmed track survives unseen
        p("other_person_radius", 0.6)
        # ID stability (person_tracking.Tracker): see there for each one's reason
        p("high_score", 0.45)              # >= : may start tracks; below: only extend them
        p("iou_strong", 0.4)
        p("dispute_frames", 3)
        p("dup_iou", 0.5)
        p("min_hits", 3)
        p("tentative_max_missed", 2)
        # never lose a true target / re-identify (person_tracking.Tracker)
        p("isolated_hold_timeout", 30.0)
        p("near_range", 1.0)
        p("near_box_fraction", 0.85)       # box >= this x image height = too close for the LiDAR
        p("reid_window", 30.0)
        p("reid_sim", 0.66)
        p("reid_margin", 0.08)
        p("reid_frames", 3)
        p("select_requires_range", True)   # False only for UI tests without a LiDAR
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.tracker = Tracker(
            iou_min=float(g("iou_min")), new_track_score=float(g("new_track_score")),
            max_missed=int(g("max_missed")), track_gate=float(g("track_gate")),
            max_person_speed=float(g("max_person_speed")),
            lidar_only_timeout=float(g("lidar_only_timeout")),
            min_track_points=int(g("min_track_points")),
            max_cluster_width=float(g("max_cluster_width")),
            min_range=float(g("min_range")), max_range=float(g("max_range")),
            other_person_radius=float(g("other_person_radius")),
            high_score=float(g("high_score")), iou_strong=float(g("iou_strong")),
            dispute_frames=int(g("dispute_frames")), dup_iou=float(g("dup_iou")),
            min_hits=int(g("min_hits")), tentative_max_missed=int(g("tentative_max_missed")),
            isolated_hold_timeout=float(g("isolated_hold_timeout")),
            near_range=float(g("near_range")), reid_window=float(g("reid_window")),
            reid_sim=float(g("reid_sim")), reid_margin=float(g("reid_margin")),
            reid_frames=int(g("reid_frames")),
            select_requires_range=bool(g("select_requires_range")))
        self.near_frac = float(g("near_box_fraction"))
        self.last_relink = None
        self.base_frame, self.odom_frame = g("base_frame"), g("odom_frame")
        self.rot = int(g("image_rotation_deg")) % 360
        self.publish_target = bool(g("publish_target"))
        self.win_scale = float(g("angular_window_scale"))
        self.win_min = math.radians(float(g("min_window_deg")))
        self.win_max = math.radians(float(g("max_window_deg")))
        self.cluster_depth = float(g("cluster_depth"))
        self.min_cluster = int(g("min_cluster_points"))
        self.scan_max_age = float(g("scan_max_age"))
        self.odom_timeout = float(g("odom_timeout"))

        self.K = None
        self.width, self.height = 640, 480
        self.scan, self.scan_time = None, 0.0
        self.odom = None                         # (tx, ty, yaw, monotonic receipt)
        self.last_state = None
        self.pending_state, self.pending_n = None, 0
        self.last_refusal = None
        self.proc_ms, self.lag_ms = [], []
        self.stats = {"no_range": 0, "held": 0, "targets": 0, "not_person_sized": 0}

        self.tf_buffer = tf2_ros.Buffer()
        self.create_subscription(
            TFMessage, "/tf_static", self.on_tf_static,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.tracks_pub = self.create_publisher(PersonTracks, g("tracks_topic"), 10)
        self.target_pub = self.create_publisher(PoseStamped, g("target_topic"), 10)
        self.id_pub = self.create_publisher(Int32, "/follow/track_id", 10)
        self.source_pub = self.create_publisher(String, "/follow/target_source", 10)
        self.hint_pub = self.create_publisher(PoseStamped, "/follow/search_hint", 10)
        self.create_subscription(CameraInfo, g("camera_info_topic"), self.on_info, be)
        self.create_subscription(LaserScan, g("scan_topic"), self.on_scan, be)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, be)
        self.create_subscription(PersonDetections, g("detections_topic"), self.on_dets, 10)
        self.create_subscription(Int32, g("select_topic"), self.on_select, 10)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"person_tracker: {g('detections_topic')} + {g('scan_topic')} -> {g('tracks_topic')}"
            + (f"; selected person -> {g('target_topic')}" if self.publish_target
               else " (target_selector owns /follow/target)"))

    # ---------------- inputs ----------------
    def on_info(self, m):
        if self.K is None:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])            # fx, fy, cx, cy
        if m.width and m.height:
            self.width, self.height = m.width, m.height

    def on_tf_static(self, m):
        for t in m.transforms:
            self.tf_buffer.set_transform_static(t, "tf_static")

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
        if m.data < 0:
            self.tracker.clear()
            self.get_logger().info("selection cleared")
        elif self.tracker.select(m.data):
            t = self.tracker.selected()
            where = (f" at {t.range:.1f} m, {math.degrees(t.bearing):+.0f} deg"
                     if t.range == t.range else " (no LiDAR range yet)")
            self.get_logger().info(f"SELECTED person #{m.data}{where}")
        else:
            known = any(t.id == m.data for t in self.tracker.confirmed())
            self.get_logger().warn(
                f"select #{m.data} ignored: " + ("never ranged by the LiDAR -- not confirmed as "
                                                  "a person" if known else "no such person"))

    # ---------------- geometry ----------------
    def camera_ray(self, u, v, stamp, frame):
        """Pixel -> (camera origin xy, bearing) in base_frame, or None."""
        fx, fy, cx, cy = self.K
        d_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, stamp,
                                                 timeout=Duration(seconds=0.0))
        except tf2_ros.TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, frame, rclpy.time.Time())
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f"no TF {self.base_frame}<-{frame}: {str(e)[:80]}",
                                       throttle_duration_sec=5.0)
                return None
        t = tf.transform.translation
        d_base = quat_rotate(tf.transform.rotation, d_opt)
        return (t.x, t.y), math.atan2(d_base[1], d_base[0])

    # ---------------- per frame ----------------
    def on_dets(self, m):
        t0 = time.perf_counter()
        now = time.monotonic()
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.lag_ms.append(1000 * (self.get_clock().now().nanoseconds * 1e-9 - st))

        scan_ok = self.scan is not None and now - self.scan_time <= self.scan_max_age
        pts = (scan_xy(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                       self.scan.range_min, self.scan.range_max) if scan_ok else None)
        T = (self.odom[:3] if self.odom is not None and now - self.odom[3] <= self.odom_timeout
             else None)
        stamp = rclpy.time.Time.from_msg(m.header.stamp)

        img_h = m.image_height or self.height
        dets = []
        for d in m.detections:
            cx, cy = (d.x1 + d.x2) / 2.0, (d.y1 + d.y2) / 2.0
            w, h = d.x2 - d.x1, d.y2 - d.y1
            score = d.score
            bearing, xy = NAN, None
            if self.K is not None:
                ray = self.camera_ray(cx, cy, stamp, m.header.frame_id)
                if ray is not None:
                    origin, bearing = ray
                    if pts is not None:
                        half = min(self.win_max, max(self.win_min,
                                                     math.atan((w / 2.0) / self.K[0]) * self.win_scale))
                        xy = range_at(pts[0], pts[1], origin, bearing, half,
                                      self.cluster_depth, self.min_cluster)
                        if xy is None:
                            self.stats["no_range"] += 1
                        elif not person_sized((d.x1, d.y1, d.x2, d.y2),
                                              math.hypot(xy[0] - origin[0], xy[1] - origin[1]),
                                              self.K[0], self.K[1], img_h):
                            xy = None                   # that range is not this person's
                            self.stats["not_person_sized"] += 1
            emb = np.frombuffer(d.embedding.tobytes(), np.float32) if len(d.embedding) else None
            dets.append(Detection((d.x1, d.y1, d.x2, d.y2), score, bearing, xy, emb,
                                  near=h >= self.near_frac * img_h))

        tracks, target = self.tracker.update(dets, now, T, pts)
        state = self.tracker.state()
        if self.tracker.relinked is not None and self.tracker.relinked != self.last_relink:
            self.last_relink = self.tracker.relinked
            self.get_logger().warn(
                f"RE-IDENTIFIED person #{self.tracker.selected_id} (appearance "
                f"{self.tracker.relinked[2]:.2f}; was shown as #{self.tracker.relinked[1]})")
        self.log_changes(state)
        if target is not None and self.publish_target:
            self.publish(m.header.stamp, target)
        elif self.publish_target and T is not None:
            hint = self.tracker.search_hint(now)
            if hint is not None:
                hx, hy = to_base(T, *hint)
                hp = PoseStamped()
                hp.header.stamp, hp.header.frame_id = m.header.stamp, self.base_frame
                hp.pose.position.x, hp.pose.position.y = float(hx), float(hy)
                hp.pose.orientation.w = 1.0
                self.hint_pub.publish(hp)
        self.publish_tracks(m.header, tracks, state)
        self.proc_ms.append(1000 * (time.perf_counter() - t0))
        if self.K is None:
            self.get_logger().warn("waiting for camera_info -- no bearings or ranges yet",
                                   throttle_duration_sec=5.0)
        elif not scan_ok:
            self.get_logger().warn("scan stale or missing -- no ranges, no target",
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
        # different person before it receives that person's position.
        self.id_pub.publish(Int32(data=int(self.tracker.selected_id)))
        self.source_pub.publish(String(data=source))
        self.target_pub.publish(out)
        self.stats["targets"] += 1
        self.stats["held"] += source == "lidar"

    def publish_tracks(self, header, tracks, state):
        out = PersonTracks()
        out.header = header
        out.image_width, out.image_height = int(self.width), int(self.height)
        out.image_rotation_deg = self.rot
        out.odom_frame = self.odom_frame
        out.selected_id = int(self.tracker.selected_id)
        out.target_state = state
        for t in tracks:
            p = PersonTrack()
            p.id = int(t.id)
            p.x1, p.y1, p.x2, p.y2 = (float(v) for v in t.box)
            p.score = float(t.score)
            p.visible = bool(t.visible)
            p.age_frames, p.missed_frames = int(t.age), int(t.missed)
            p.bearing_deg = math.degrees(t.bearing) if t.bearing == t.bearing else NAN
            p.range_m = float(t.range)
            if t.pos is not None:
                p.position.x, p.position.y, p.position.z = float(t.pos[0]), float(t.pos[1]), 0.0
            else:
                p.position.x = p.position.y = p.position.z = NAN
            sel = self.tracker.selected()
            p.appearance_sim = (float(np.dot(t.recent[-1], sel.gallery))
                                if t.recent and sel is not None and sel.gallery is not None else NAN)
            p.ranged = bool(t.ever_ranged)
            out.tracks.append(p)
        self.tracks_pub.publish(out)

    def log_changes(self, state):
        # Debounce: a state is logged once it has held for 3 frames (LOST/NONE at
        # once) -- the live log flapped TRACKING/HOLD every 100-300 ms.
        if state == self.pending_state:
            self.pending_n += 1
        else:
            self.pending_state, self.pending_n = state, 1
        if state != self.last_state and (self.pending_n >= 3
                                         or state in ("LOST", "NONE", "SEARCHING")):
            sid = self.tracker.selected_id
            msgs = {"TRACKING": f"person #{sid}: camera has them",
                    "HOLD_LIDAR": f"person #{sid}: camera lost them -- holding on the LiDAR",
                    "COASTING": f"person #{sid}: not seen for 3+ frames",
                    "SEARCHING": f"person #{sid} LOST"
                                 + (f" (LiDAR hold refused: {self.tracker.hold_refusal})"
                                    if self.tracker.hold_refusal else "")
                                 + f" -- searching: re-identification for "
                                 f"{self.tracker.reid_window:.0f} s",
                    "LOST": f"person #{sid} LOST"
                            + (f" (LiDAR hold refused: {self.tracker.hold_refusal})"
                               if self.tracker.hold_refusal else "")
                            + " -- no target until someone is selected again",
                    "NONE": "nobody selected"}
            # One call site per severity: rclpy raises "Logger severity cannot be
            # changed between calls" if one line logs at two levels -- that crashed
            # this node at the first state change (scene test, 2026-09-29).
            if self.last_state is not None or state != "NONE":
                if state in ("LOST", "HOLD_LIDAR", "SEARCHING"):
                    self.get_logger().warn(msgs[state])
                else:
                    self.get_logger().info(msgs[state])
            self.last_state = state
        r = self.tracker.hold_refusal
        if r and r != self.last_refusal and state not in ("LOST", "SEARCHING"):   # LOST line has it
            self.get_logger().warn(f"LiDAR hold refused: {r}")
        self.last_refusal = r

    def report(self):
        if self.proc_ms:
            p, lag = sorted(self.proc_ms), sorted(self.lag_ms)
            s = self.stats
            self.get_logger().info(
                f"{len(p) / 10.0:.1f} Hz, {len(self.tracker.tracks)} person(s), "
                f"selected #{self.tracker.selected_id} {self.tracker.state()}; "
                f"{s['targets']} targets ({s['held']} LiDAR-held), {s['no_range']} unranged, "
                f"{s['not_person_sized']} ranges refused (not person-sized); "
                f"update median {p[len(p) // 2]:.1f} ms max {p[-1]:.1f} ms; "
                f"detections {lag[len(lag) // 2]:.0f} ms old on arrival")
        self.proc_ms.clear()
        self.lag_ms.clear()
        for k in self.stats:
            self.stats[k] = 0


def main():
    rclpy.init()
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
