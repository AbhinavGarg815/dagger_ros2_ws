#!/usr/bin/env python3
"""Every detected person -> a stable ID, for click-to-select following.

    /follow/detections (Detection2DArray, person_detector)  -> /follow/tracks (dagger_msgs/PersonTracks)
    /follow/select (Int32: track id, -1 = clear)             -> selection kept here

One PersonTracks message per camera frame, stamped with THAT frame, so the
browser UI (ui_server) can pair it with the frame's JPEG and draw boxes that
belong exactly to the picture the operator clicks. Tracking logic lives in
person_tracking.py (no ROS; unit-tested by tools/test_person_tracking.py).

Phase 1: image-space tracking (IoU + Hungarian). The LiDAR/odom position and the
appearance term come in the next phases; until then bearing/range are NaN.
"""

import time

import rclpy
from dagger_msgs.msg import PersonTrack, PersonTracks
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int32
from vision_msgs.msg import Detection2DArray

from dagger_bringup.person_tracking import Tracker

NAN = float("nan")


class PersonTracker(Node):

    def __init__(self):
        super().__init__("person_tracker")
        self.declare_parameter("detections_topic", "/follow/detections")
        self.declare_parameter("camera_info_topic", "/d435/color/camera_info")
        self.declare_parameter("tracks_topic", "/follow/tracks")
        self.declare_parameter("select_topic", "/follow/select")
        self.declare_parameter("image_rotation_deg", 0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("iou_min", 0.2)
        self.declare_parameter("new_track_score", 0.5)
        self.declare_parameter("max_missed", 10)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.tracker = Tracker(iou_min=float(g("iou_min")),
                               new_track_score=float(g("new_track_score")),
                               max_missed=int(g("max_missed")))
        self.rot = int(g("image_rotation_deg")) % 360
        self.odom_frame = g("odom_frame")
        self.width, self.height = 640, 480

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(PersonTracks, g("tracks_topic"), 10)
        self.create_subscription(Detection2DArray, g("detections_topic"), self.on_dets, 10)
        self.create_subscription(CameraInfo, g("camera_info_topic"), self.on_info, be)
        self.create_subscription(Int32, g("select_topic"), self.on_select, 10)
        self.proc_ms, self.lag_ms = [], []
        self.create_timer(10.0, self.report)
        self.get_logger().info(f"person_tracker: {g('detections_topic')} -> {g('tracks_topic')}")

    def report(self):
        """Every 10 s: frames, time spent here, and how late detections arrive."""
        if self.proc_ms:
            p, lag = sorted(self.proc_ms), sorted(self.lag_ms)
            self.get_logger().info(
                f"{len(p) / 10.0:.1f} Hz, {len(self.tracker.tracks)} track(s); update "
                f"median {p[len(p) // 2]:.1f} ms max {p[-1]:.1f} ms; detection age on "
                f"arrival median {lag[len(lag) // 2]:.0f} ms")
        self.proc_ms.clear()
        self.lag_ms.clear()

    def on_info(self, m):
        if m.width and m.height:
            self.width, self.height = m.width, m.height

    def on_select(self, m):
        if m.data < 0:
            self.tracker.clear()
            self.get_logger().info("selection cleared")
        elif self.tracker.select(m.data):
            self.get_logger().info(f"selected person #{m.data}")
        else:
            self.get_logger().warn(f"select #{m.data} ignored: no such track")

    def on_dets(self, m):
        t0 = time.perf_counter()
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.lag_ms.append(1000 * (self.get_clock().now().nanoseconds * 1e-9 - st))
        boxes, scores = [], []
        for d in m.detections:
            cx, cy = d.bbox.center.position.x, d.bbox.center.position.y
            w, h = d.bbox.size_x, d.bbox.size_y
            boxes.append((cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2))
            scores.append(d.results[0].hypothesis.score if d.results else 0.0)
        tracks = self.tracker.update(boxes, scores)

        out = PersonTracks()
        out.header = m.header
        out.image_width, out.image_height = int(self.width), int(self.height)
        out.image_rotation_deg = self.rot
        out.odom_frame = self.odom_frame
        out.selected_id = int(self.tracker.selected_id)
        out.target_state = self.tracker.state()
        for t in tracks:
            p = PersonTrack()
            p.id = int(t.id)
            p.x1, p.y1, p.x2, p.y2 = (float(v) for v in t.box)
            p.score = float(t.score)
            p.visible = bool(t.visible)
            p.age_frames = int(t.age)
            p.missed_frames = int(t.missed)
            p.bearing_deg = p.range_m = p.appearance_sim = NAN
            p.position.x = p.position.y = p.position.z = NAN
            out.tracks.append(p)
        self.pub.publish(out)
        self.proc_ms.append(1000 * (time.perf_counter() - t0))


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
