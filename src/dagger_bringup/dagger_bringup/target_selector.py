#!/usr/bin/env python3
"""Pick one person to follow and locate them: camera gives the bearing, LiDAR the range.

    /follow/detections + /d435/color/camera_info + /scan_safety
        -> /follow/target   (geometry_msgs/PoseStamped, base_footprint)

WHY THE RANGE DOES NOT COME FROM THE CAMERA. This is a D435, whose stereo depth
washes out in direct sunlight, and the rover is outdoor-only. The camera is good
at "which direction", the LiDAR is good at "how far", so each is used for what it
is good at. Depth is not even enabled in camera.launch.py.

HOW THE TWO ARE FUSED. A bbox centre plus the colour intrinsics gives a ray in
the camera optical frame. That ray is rotated into base_footprint through TF at
the image's own capture stamp, which also yields the camera's position -- and
the camera is NOT on the centreline: the D435's colour imager sits 14.7 mm left
of its mounting frame, so the bearing is measured from the camera's real origin
rather than from the robot's. /scan_safety points are then taken as seen FROM
that origin, and those falling inside the bbox's angular width are the target.

The published pose is in base_footprint, because that is what a controller wants:
position and bearing relative to the rover, not to the lens.

NO RANGE, NO TARGET. If the LiDAR cannot see the person the node publishes
nothing rather than a bearing with a guessed distance. A standing adult is
visible to /scan_safety from about 1 m out (see collision_monitor.yaml for the
near-field blind cone), which is well inside the ~3 m follow distance.

This node selects and locates. It commands no motion.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, LaserScan
from vision_msgs.msg import Detection2DArray


def quat_rotate(q, v):
    """Rotate vector v by quaternion q (geometry_msgs Quaternion)."""
    u = np.array([q.x, q.y, q.z])
    s = q.w
    return 2.0 * np.dot(u, v) * u + (s * s - np.dot(u, u)) * v + 2.0 * s * np.cross(u, v)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class TargetSelector(Node):

    def __init__(self):
        super().__init__("target_selector")

        self.declare_parameter("detections_topic", "/follow/detections")
        self.declare_parameter("camera_info_topic", "/d435/color/camera_info")
        self.declare_parameter("scan_topic", "/scan_safety")
        self.declare_parameter("target_topic", "/follow/target")
        self.declare_parameter("base_frame", "base_footprint")
        # "centred" keeps following whoever is ahead; "nearest" grabs the closest
        # person. Both are only tie-breakers -- an already-tracked target wins.
        self.declare_parameter("selection_mode", "centred")
        self.declare_parameter("min_score", 0.5)
        self.declare_parameter("min_range", 1.0)
        self.declare_parameter("max_range", 8.0)
        # A detection within this angle of the previous target is treated as the
        # same person, so the rover does not swap targets when someone walks past.
        self.declare_parameter("association_gate_deg", 20.0)
        # Scan points are matched inside the bbox's own angular width, scaled a
        # little since a box hugs the body while the legs spread wider.
        self.declare_parameter("angular_window_scale", 1.2)
        self.declare_parameter("min_window_deg", 2.0)
        self.declare_parameter("max_window_deg", 25.0)
        # Points within this much of the nearest one count as the same body.
        self.declare_parameter("cluster_depth", 0.6)
        self.declare_parameter("min_cluster_points", 2)
        self.declare_parameter("scan_max_age", 0.4)
        self.declare_parameter("lost_timeout", 1.5)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base_frame = g("base_frame")
        self.mode = g("selection_mode")
        self.min_score = float(g("min_score"))
        self.min_range = float(g("min_range"))
        self.max_range = float(g("max_range"))
        self.gate = math.radians(float(g("association_gate_deg")))
        self.win_scale = float(g("angular_window_scale"))
        self.win_min = math.radians(float(g("min_window_deg")))
        self.win_max = math.radians(float(g("max_window_deg")))
        self.cluster_depth = float(g("cluster_depth"))
        self.min_cluster = int(g("min_cluster_points"))
        self.scan_max_age = float(g("scan_max_age"))
        self.lost_timeout = float(g("lost_timeout"))

        self.K = None
        self.scan = None
        self.scan_time = 0.0
        self.last_bearing = None
        self.last_target_time = 0.0
        self.stats = {"msgs": 0, "published": 0, "no_range": 0, "no_det": 0, "tf_fallback": 0}

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(PoseStamped, g("target_topic"), 10)
        self.create_subscription(CameraInfo, g("camera_info_topic"), self.on_info, be)
        self.create_subscription(LaserScan, g("scan_topic"), self.on_scan, be)
        self.create_subscription(Detection2DArray, g("detections_topic"), self.on_dets, 10)
        self.create_timer(10.0, self.report)

        self.get_logger().info(
            f"target_selector: mode={self.mode}, follow range {self.min_range}-{self.max_range} m, "
            f"bearing from {g('camera_info_topic')}, range from {g('scan_topic')}")

    def on_info(self, msg):
        if self.K is None:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])   # fx, fy, cx, cy
            self.get_logger().info(
                f"intrinsics fx {self.K[0]:.1f} fy {self.K[1]:.1f} "
                f"cx {self.K[2]:.1f} cy {self.K[3]:.1f} -> "
                f"{math.degrees(2 * math.atan(msg.width / 2 / msg.k[0])):.1f} deg horizontal FOV")

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def bearing_of(self, u, v, stamp, frame):
        """Ray through pixel (u, v) -> (camera origin xy, bearing) in base_frame."""
        fx, fy, cx, cy = self.K
        d_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, stamp,
                                                 timeout=Duration(seconds=0.05))
        except Exception:
            # The image stamp can fall outside the buffer on startup or after a
            # clock hiccup; latest is a better answer than none, but count it.
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, frame,
                                                     rclpy.time.Time(),
                                                     timeout=Duration(seconds=0.05))
                self.stats["tf_fallback"] += 1
            except Exception as e:
                self.get_logger().warn(f"no TF {self.base_frame}<-{frame}: {str(e)[:80]}",
                                       throttle_duration_sec=5.0)
                return None
        t = tf.transform.translation
        d_base = quat_rotate(tf.transform.rotation, d_opt)
        return np.array([t.x, t.y]), math.atan2(d_base[1], d_base[0])

    def range_at(self, origin, bearing, half_width):
        """Nearest LiDAR cluster inside +-half_width of `bearing`, seen from `origin`."""
        s = self.scan
        r = np.asarray(s.ranges, dtype=np.float32)
        ok = np.isfinite(r) & (r >= s.range_min) & (r <= s.range_max)
        if not ok.any():
            return None
        idx = np.nonzero(ok)[0]
        ang = s.angle_min + idx * s.angle_increment
        # Scan points are in base_frame about ITS origin; re-reference to the camera.
        px, py = r[idx] * np.cos(ang), r[idx] * np.sin(ang)
        rel = np.arctan2(py - origin[1], px - origin[0])
        d = np.abs(np.arctan2(np.sin(rel - bearing), np.cos(rel - bearing)))
        sel = d <= half_width
        if not sel.any():
            return None
        px, py = px[sel], py[sel]
        rng = np.hypot(px, py)                       # distance from the ROVER
        near = rng.min()
        body = rng <= near + self.cluster_depth      # one person, not the wall behind
        if body.sum() < self.min_cluster:
            return None
        return float(np.median(px[body])), float(np.median(py[body]))

    def on_dets(self, msg):
        self.stats["msgs"] += 1
        if self.K is None:
            self.get_logger().warn("waiting for camera_info", throttle_duration_sec=5.0)
            return
        if self.scan is None or time.monotonic() - self.scan_time > self.scan_max_age:
            self.get_logger().warn("scan stale or missing -- no target",
                                   throttle_duration_sec=5.0)
            return

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        cands = []
        for d in msg.detections:
            if not d.results or d.results[0].hypothesis.score < self.min_score:
                continue
            got = self.bearing_of(d.bbox.center.position.x, d.bbox.center.position.y,
                                  stamp, msg.header.frame_id)
            if got is None:
                return
            origin, bearing = got
            # bbox half-width in angle, from the same intrinsics
            fx = self.K[0]
            half = min(self.win_max,
                       max(self.win_min,
                           math.atan((d.bbox.size_x / 2.0) / fx) * self.win_scale))
            xy = self.range_at(origin, bearing, half)
            if xy is None:
                self.stats["no_range"] += 1
                continue
            rng = math.hypot(xy[0], xy[1])
            if not (self.min_range <= rng <= self.max_range):
                continue
            cands.append({"xy": xy, "bearing": bearing, "range": rng,
                          "score": d.results[0].hypothesis.score})

        if not cands:
            self.stats["no_det"] += 1
            if (self.last_bearing is not None
                    and time.monotonic() - self.last_target_time > self.lost_timeout):
                self.get_logger().info("target lost")
                self.last_bearing = None
            return

        # Stay with whoever we were already following, if anyone matches.
        pick = None
        if self.last_bearing is not None:
            near = min(cands, key=lambda c: abs(wrap(c["bearing"] - self.last_bearing)))
            if abs(wrap(near["bearing"] - self.last_bearing)) <= self.gate:
                pick = near
        if pick is None:
            pick = (min(cands, key=lambda c: c["range"]) if self.mode == "nearest"
                    else min(cands, key=lambda c: abs(c["bearing"])))

        # Bearing to the located point from the ROVER, which is what steers it.
        b_base = math.atan2(pick["xy"][1], pick["xy"][0])
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame
        out.pose.position.x, out.pose.position.y = pick["xy"]
        out.pose.orientation.z = math.sin(b_base / 2.0)
        out.pose.orientation.w = math.cos(b_base / 2.0)
        self.pub.publish(out)

        self.last_bearing = pick["bearing"]
        self.last_target_time = time.monotonic()
        self.stats["published"] += 1

    def report(self):
        s = self.stats
        if s["msgs"]:
            self.get_logger().info(
                f"{s['msgs'] / 10.0:.1f} Hz in, {s['published']} targets, "
                f"{s['no_det']} frames with no usable person, {s['no_range']} detections "
                f"the LiDAR could not range"
                + (f", {s['tf_fallback']} TF stamp fallbacks" if s["tf_fallback"] else ""))
        for k in s:
            s[k] = 0


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
