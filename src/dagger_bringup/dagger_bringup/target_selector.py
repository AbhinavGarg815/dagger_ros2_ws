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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Int32, String
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
        # 8 -> 15 m: a 1.7 m person at 15 m is still ~45 px tall in the 416 px
        # network input, well inside what YOLOX-Nano detects. 8 m was a number
        # chosen here, not a sensor limit, and the 16:47 run hit it exactly and
        # dropped the target.
        self.declare_parameter("max_range", 15.0)
        # A detection within this angle of the previous target is treated as the
        # same person, so the rover does not swap targets when someone walks past.
        self.declare_parameter("association_gate_deg", 20.0)
        # Association also has to match in RANGE. Bearing alone let a person
        # crossing at a similar angle but a very different distance steal the
        # lock -- the range was already being computed and simply not used.
        self.declare_parameter("association_range_gate", 1.5)
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

        # ---- LiDAR track: keep the person when the camera cannot see them ----
        # The camera CONFIRMS who the person is; the 360 deg LiDAR KEEPS them. A
        # confirmed target is tracked in the odom frame, and while the camera has
        # no match the track is carried forward on the LiDAR cluster nearest its
        # predicted position -- beside or even behind the rover, where the 55.7
        # deg camera cannot see. The LiDAR cannot tell a person from a post, so
        # every guard below exists to stop the track sliding onto something else.
        self.declare_parameter("lidar_track", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("odom_timeout", 0.5)
        # How long the track may run on LiDAR alone before the camera must
        # reconfirm it. Past this it is treated as lost.
        self.declare_parameter("lidar_only_timeout", 5.0)
        # Association radius around the predicted position, plus what a walking
        # person can cover since the last update.
        self.declare_parameter("track_gate", 0.6)
        self.declare_parameter("max_person_speed", 1.8)
        # A person seen by a 2D scan is a short arc. Anything wider is a wall,
        # a vehicle or a hedge, and is rejected rather than followed.
        self.declare_parameter("max_cluster_width", 0.9)
        self.declare_parameter("min_track_points", 3)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base_frame = g("base_frame")
        self.mode = g("selection_mode")
        self.min_score = float(g("min_score"))
        self.min_range = float(g("min_range"))
        self.max_range = float(g("max_range"))
        self.gate = math.radians(float(g("association_gate_deg")))
        self.range_gate = float(g("association_range_gate"))
        self.win_scale = float(g("angular_window_scale"))
        self.win_min = math.radians(float(g("min_window_deg")))
        self.win_max = math.radians(float(g("max_window_deg")))
        self.cluster_depth = float(g("cluster_depth"))
        self.min_cluster = int(g("min_cluster_points"))
        self.scan_max_age = float(g("scan_max_age"))
        self.lost_timeout = float(g("lost_timeout"))
        self.lidar_track = bool(g("lidar_track"))
        self.odom_frame = g("odom_frame")
        self.lidar_only_timeout = float(g("lidar_only_timeout"))
        self.track_gate = float(g("track_gate"))
        self.max_person_speed = float(g("max_person_speed"))
        self.max_cluster_width = float(g("max_cluster_width"))
        self.min_track_points = int(g("min_track_points"))

        self.K = None
        self.scan = None
        self.scan_time = 0.0
        self.last_bearing = None
        self.last_range = None
        self.last_target_time = 0.0
        # Increments on every acquisition that is NOT a continuation of the
        # previous one, so a silent swap between people becomes visible.
        self.track_id = 0
        # Odom-frame track: pos (x, y), vel (vx, vy), t (monotonic) of the last
        # update, t_cam of the last CAMERA confirmation, source of the last fix.
        self.trk = None
        self.stats = {"msgs": 0, "published": 0, "no_range": 0, "no_det": 0, "tf_fallback": 0,
                      "lidar": 0}

        # No TransformListener: in Python it decodes every /tf message (a Python
        # TF listener cost ~16-25 % CPU elsewhere, measured 2026-09-29). This node
        # needs only base<-camera, which is STATIC (fixed URDF joint), and
        # odom<-base, which is exactly /odometry/filtered.
        self.tf_buffer = tf2_ros.Buffer()
        self.create_subscription(
            TFMessage, "/tf_static", self.on_tf_static,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom = None           # (tx, ty, yaw, monotonic receipt time)
        self.odom_timeout = float(g("odom_timeout"))

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(PoseStamped, g("target_topic"), 10)
        self.track_pub = self.create_publisher(Int32, "/follow/track_id", 10)
        # "camera" or "lidar": which sensor produced the current target.
        self.source_pub = self.create_publisher(String, "/follow/target_source", 10)
        self.create_subscription(CameraInfo, g("camera_info_topic"), self.on_info, be)
        self.create_subscription(LaserScan, g("scan_topic"), self.on_scan, be)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, be)
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

    def on_tf_static(self, msg):
        for t in msg.transforms:
            self.tf_buffer.set_transform_static(t, "tf_static")

    def on_odom(self, msg):
        if msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.base_frame:
            self.get_logger().warn(
                f"odometry is {msg.header.frame_id}->{msg.child_frame_id}, expected "
                f"{self.odom_frame}->{self.base_frame}; ignoring it", throttle_duration_sec=10.0)
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw, time.monotonic())

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

    def odom_T_base(self):
        """(tx, ty, yaw) of base_frame in odom_frame from the latest odometry."""
        if self.odom is None or time.monotonic() - self.odom[3] > self.odom_timeout:
            self.get_logger().warn(
                f"no fresh {self.odom_frame}<-{self.base_frame} odometry "
                f"(timeout {self.odom_timeout}s) -- LiDAR tracking off this cycle",
                throttle_duration_sec=5.0)
            return None
        return self.odom[:3]

    @staticmethod
    def to_odom(T, x, y):
        tx, ty, yaw = T
        c, s_ = math.cos(yaw), math.sin(yaw)
        return tx + c * x - s_ * y, ty + s_ * x + c * y

    @staticmethod
    def to_base(T, x, y):
        tx, ty, yaw = T
        c, s_ = math.cos(yaw), math.sin(yaw)
        dx, dy = x - tx, y - ty
        return c * dx + s_ * dy, -s_ * dx + c * dy

    def track_update(self, T, xy_base, source):
        """Fold a new fix (base frame) into the odom-frame track."""
        now = time.monotonic()
        ox, oy = self.to_odom(T, *xy_base)
        if self.trk is None:
            self.trk = {"pos": (ox, oy), "vel": (0.0, 0.0), "t": now, "t_cam": now,
                        "src": source}
        else:
            dt = max(1e-3, now - self.trk["t"])
            px, py = self.trk["pos"]
            vx, vy = (ox - px) / dt, (oy - py) / dt
            sp = math.hypot(vx, vy)
            if sp > self.max_person_speed:          # a jump, not walking
                vx, vy = vx * self.max_person_speed / sp, vy * self.max_person_speed / sp
            a = 0.3                                 # light smoothing on velocity
            ovx, ovy = self.trk["vel"]
            self.trk["vel"] = (a * vx + (1 - a) * ovx, a * vy + (1 - a) * ovy)
            self.trk["pos"] = (ox, oy)
            self.trk["t"] = now
            if source == "camera":
                self.trk["t_cam"] = now
        if self.trk["src"] != source:
            if source == "lidar":
                self.get_logger().warn("camera lost the target -- holding it on the LiDAR "
                                       f"(up to {self.lidar_only_timeout:.0f} s)")
            else:
                self.get_logger().info("camera reconfirmed the target")
            self.trk["src"] = source

    def lidar_fix(self, T):
        """Find the tracked person in the scan near the predicted position.

        Returns an (x, y) in base_frame, or None. None is the safe answer:
        anything ambiguous ends the LiDAR hold rather than guessing.
        """
        if self.trk is None or self.scan is None:
            return None
        now = time.monotonic()
        if now - self.trk["t_cam"] > self.lidar_only_timeout:
            return None
        dt = now - self.trk["t"]
        px = self.trk["pos"][0] + self.trk["vel"][0] * dt
        py = self.trk["pos"][1] + self.trk["vel"][1] * dt
        gate = self.track_gate + min(self.max_person_speed * dt, 1.0)

        sc = self.scan
        r = np.asarray(sc.ranges, dtype=np.float32)
        ok = np.isfinite(r) & (r >= sc.range_min) & (r <= sc.range_max)
        if not ok.any():
            return None
        idx = np.nonzero(ok)[0]
        ang = sc.angle_min + idx * sc.angle_increment
        bx, by = r[idx] * np.cos(ang), r[idx] * np.sin(ang)
        tx, ty, yaw = T
        c, s_ = math.cos(yaw), math.sin(yaw)
        ox, oy = tx + c * bx - s_ * by, ty + s_ * bx + c * by
        d = np.hypot(ox - px, oy - py)
        sel = d <= gate
        if sel.sum() < self.min_track_points:
            return None
        # The body: points close to the one nearest the prediction.
        k = np.argmin(np.where(sel, d, np.inf))
        body = sel & (np.hypot(ox - ox[k], oy - oy[k]) <= self.max_cluster_width / 2 + 0.1)
        if body.sum() < self.min_track_points:
            return None
        bxs, bys = ox[body], oy[body]
        width = math.hypot(bxs.max() - bxs.min(), bys.max() - bys.min())
        # A wide return straddling the gate is a wall or a vehicle. Refuse it.
        near_all = sel & (np.hypot(ox - ox[k], oy - oy[k]) <= 1.5)
        span = math.hypot(ox[near_all].max() - ox[near_all].min(),
                          oy[near_all].max() - oy[near_all].min())
        if width > self.max_cluster_width or span > 2.0 * self.max_cluster_width:
            self.get_logger().warn(
                f"LiDAR cluster near the track is {max(width, span):.2f} m wide -- not "
                f"a person, refusing to follow it", throttle_duration_sec=2.0)
            return None
        mx, my = float(np.median(bxs)), float(np.median(bys))
        return self.to_base(T, mx, my)

    def publish_target(self, stamp, xy, source):
        b_base = math.atan2(xy[1], xy[0])
        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.base_frame
        out.pose.position.x, out.pose.position.y = float(xy[0]), float(xy[1])
        out.pose.orientation.z = math.sin(b_base / 2.0)
        out.pose.orientation.w = math.cos(b_base / 2.0)
        # Id first: the controller disables on an id change, and should learn of
        # a new person before it receives that person's position.
        self.track_pub.publish(Int32(data=self.track_id))
        self.source_pub.publish(String(data=source))
        self.pub.publish(out)
        # Bearing from the rover is a fine stand-in for the camera bearing in
        # the association gate: the camera sits on the rover's x axis.
        self.last_bearing = b_base
        self.last_range = math.hypot(xy[0], xy[1])
        self.last_target_time = time.monotonic()
        self.stats["published"] += 1

    def try_lidar_hold(self, stamp):
        """Carry the track on the LiDAR. True if a target was published."""
        if not self.lidar_track or self.trk is None:
            return False
        T = self.odom_T_base()
        if T is None:
            return False
        xy = self.lidar_fix(T)
        if xy is None:
            return False
        rng = math.hypot(*xy)
        if not (self.min_range <= rng <= self.max_range):
            return False
        self.track_update(T, xy, "lidar")
        self.stats["lidar"] += 1
        self.publish_target(stamp, xy, "lidar")
        return True

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
            if self.try_lidar_hold(msg.header.stamp):
                return
            if (self.last_bearing is not None
                    and time.monotonic() - self.last_target_time > self.lost_timeout):
                self.get_logger().info("target lost")
                self.last_bearing = None
                self.last_range = None
                self.trk = None
            return

        # Stay with whoever we were already following, if anyone matches in BOTH
        # bearing and range. Matching on bearing alone let someone crossing at a
        # similar angle but a very different distance take over the lock.
        pick = None
        T = self.odom_T_base() if (self.lidar_track and self.trk is not None) else None
        if T is not None:
            # Associate by PREDICTED POSITION in the odom frame. Bearing-first
            # association (below) let the lock hop between two people ~1 m apart
            # with no id change: as the tracked person walked across, a bystander
            # became the nearest-in-bearing candidate, passed the 20 deg / 1.5 m
            # gate, and was followed at 0.36 m/s (found in simulation 2026-09-26).
            dt = time.monotonic() - self.trk["t"]
            px = self.trk["pos"][0] + self.trk["vel"][0] * dt
            py = self.trk["pos"][1] + self.trk["vel"][1] * dt
            gate = self.track_gate + min(self.max_person_speed * dt, 1.0)
            best, bd = None, float("inf")
            for c in cands:
                ox, oy = self.to_odom(T, *c["xy"])
                dd = math.hypot(ox - px, oy - py)
                if dd < bd:
                    best, bd = c, dd
            if bd <= gate:
                pick = best
        elif self.last_bearing is not None:
            # No odom track (LiDAR tracking off, or no TF): bearing + range gate.
            near = min(cands, key=lambda c: abs(wrap(c["bearing"] - self.last_bearing)))
            if (abs(wrap(near["bearing"] - self.last_bearing)) <= self.gate
                    and abs(near["range"] - self.last_range) <= self.range_gate):
                pick = near
        if pick is None and self.trk is not None and self.try_lidar_hold(msg.header.stamp):
            # The camera sees somebody, but not the person being tracked, while
            # the LiDAR still has the tracked person. Stay with the track: a new
            # face in frame must not steal the lock.
            return
        if pick is None:
            pick = (min(cands, key=lambda c: c["range"]) if self.mode == "nearest"
                    else min(cands, key=lambda c: abs(c["bearing"])))
            self.trk = None                         # genuinely a new target
            self.track_id += 1
            if self.last_bearing is not None:
                # Not a continuation: the lock moved to a different person.
                self.get_logger().warn(
                    f"TARGET SWITCHED (track {self.track_id}): was "
                    f"{math.degrees(self.last_bearing):+.1f} deg / {self.last_range:.2f} m, "
                    f"now {math.degrees(pick['bearing']):+.1f} deg / {pick['range']:.2f} m "
                    f"-- {len(cands)} people in frame")
            else:
                self.get_logger().info(
                    f"target acquired (track {self.track_id}) at "
                    f"{math.degrees(pick['bearing']):+.1f} deg / {pick['range']:.2f} m")
        if self.lidar_track:
            T = self.odom_T_base()
            if T is not None:
                self.track_update(T, pick["xy"], "camera")
        self.publish_target(msg.header.stamp, pick["xy"], "camera")

    def report(self):
        s = self.stats
        if s["msgs"]:
            self.get_logger().info(
                f"{s['msgs'] / 10.0:.1f} Hz in, {s['published']} targets, "
                f"{s['no_det']} frames with no usable person, {s['no_range']} detections "
                f"the LiDAR could not range, {s['lidar']} held on LiDAR alone"
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
