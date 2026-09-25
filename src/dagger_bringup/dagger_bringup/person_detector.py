#!/usr/bin/env python3
"""Person detection for follow-me: D435 colour -> vision_msgs/Detection2DArray.

    /d435/color/image_raw --> [YOLOX FP16 on TensorRT] --> /follow/detections

Detection only. It picks no target, computes no range and commands no motion --
that is the target selector's job, and range comes from the LiDAR, not from this
camera (the D435's depth washes out in sunlight).

RATE IS DELIBERATELY THROTTLED. The camera runs at 30 Hz and the detector can
keep up (66.7 FPS measured), but the follow loop is limited by the LiDAR at
10 Hz, so running faster would just burn CPU that Nav2 sometimes needs. Frames
arriving before the next slot are dropped rather than queued, so a detection is
always from the most recent frame and never from a backlog.

Detections carry the IMAGE's header, so the stamp is the capture time and
frame_id is the camera optical frame. The selector needs both to transform a
bearing into the LiDAR's frame at the right instant.
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

from dagger_bringup.yolox_trt import YoloxTRT, COCO_PERSON


class PersonDetector(Node):

    def __init__(self):
        super().__init__("person_detector")

        self.declare_parameter("engine_path", "")
        self.declare_parameter("image_topic", "/d435/color/image_raw")
        self.declare_parameter("detections_topic", "/follow/detections")
        self.declare_parameter("conf_threshold", 0.45)
        self.declare_parameter("nms_threshold", 0.45)
        # Matches the LiDAR's 10 Hz; see the note above about not going faster.
        self.declare_parameter("target_rate", 10.0)
        # Annotated image for Foxglove. Off by default: it costs an encode and a
        # 640x480 publish per frame, and Foxglove already starves the controller.
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_image_topic", "/follow/detections_image")
        # YOLOX is not rotation invariant: on an upside-down mount a standing
        # person appears lying down and is almost never detected (measured on
        # this rover: 1 detection in 194 ticks). Rotate for INFERENCE ONLY, then
        # map boxes back to raw pixel coordinates, so the camera_info intrinsics
        # and the URDF transform stay the single source of truth for geometry.
        # This must match the roll in dagger_description's d435_joint.
        self.declare_parameter("image_rotation_deg", 0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        engine = g("engine_path")
        if not engine:
            raise RuntimeError(
                "engine_path is required -- build one with tools/fetch_model.sh")
        self.period = 1.0 / float(g("target_rate"))
        self.publish_debug = bool(g("publish_debug_image"))
        self.rot_deg = int(g("image_rotation_deg")) % 360
        if self.rot_deg not in (0, 90, 180, 270):
            raise RuntimeError(f"image_rotation_deg must be 0/90/180/270, got {self.rot_deg}")

        self.detector = YoloxTRT(engine,
                                 conf_thres=float(g("conf_threshold")),
                                 nms_thres=float(g("nms_threshold")),
                                 classes=(COCO_PERSON,))

        from cv_bridge import CvBridge
        self.bridge = CvBridge()

        # Depth 1 + best effort: we want the newest frame, never a queue. A
        # backlog here would feed the controller a stale bearing, which is worse
        # than skipping a frame.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Detection2DArray, g("detections_topic"), 10)
        self.debug_pub = (self.create_publisher(Image, g("debug_image_topic"), 1)
                          if self.publish_debug else None)
        self.create_subscription(Image, g("image_topic"), self.on_image, qos)

        self.last_run = 0.0
        self.n_frames = 0
        self.n_skipped = 0
        self.infer_ms = []
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"person_detector: {g('image_topic')} -> {g('detections_topic')} at "
            f"{g('target_rate')} Hz, conf {g('conf_threshold')}, "
            f"engine {engine.split('/')[-1]}, input {self.detector.input_hw}")

    def on_image(self, msg):
        now = time.monotonic()
        # 0.9 * period, not period: frames only arrive on the camera's 33.3 ms
        # grid, so a strict comparison rejects the frame that lands a hair early
        # and the rate quantises down a whole frame (10 Hz became 8.55 Hz). The
        # tolerance lets every 3rd frame through and holds a true 10 Hz.
        if now - self.last_run < self.period * 0.9:
            self.n_skipped += 1
            return
        self.last_run = now

        # The driver streams rgb8; YOLOX was calibrated on cv2's BGR ordering.
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        t0 = time.perf_counter()
        dets = self.detector.detect(self.rotate(frame))
        dets = [self.unrotate(d, frame.shape[1], frame.shape[0]) for d in dets]
        self.infer_ms.append((time.perf_counter() - t0) * 1e3)
        self.n_frames += 1

        out = Detection2DArray()
        out.header = msg.header           # capture time + camera optical frame
        for x1, y1, x2, y2, score, cls in dets:
            d = Detection2D()
            d.header = msg.header
            d.bbox = BoundingBox2D()
            d.bbox.center.position.x = float((x1 + x2) / 2.0)
            d.bbox.center.position.y = float((y1 + y2) / 2.0)
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = str(cls)
            h.hypothesis.score = float(score)
            d.results.append(h)
            out.detections.append(d)
        self.pub.publish(out)

        if self.debug_pub is not None:
            import cv2
            for x1, y1, x2, y2, score, _c in dets:
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"{score:.2f}", (int(x1), max(12, int(y1) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            dbg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            dbg.header = msg.header
            self.debug_pub.publish(dbg)

    def rotate(self, frame):
        if self.rot_deg == 0:
            return frame
        import cv2
        return cv2.rotate(frame, {90: cv2.ROTATE_90_CLOCKWISE,
                                  180: cv2.ROTATE_180,
                                  270: cv2.ROTATE_90_COUNTERCLOCKWISE}[self.rot_deg])

    def unrotate(self, det, w, h):
        """Map a box found in the rotated image back to raw pixel coordinates."""
        x1, y1, x2, y2, score, cls = det
        if self.rot_deg == 0:
            return det
        if self.rot_deg == 180:
            return (w - x2, h - y2, w - x1, h - y1, score, cls)
        # A 90/270 rotation swaps the axes: the rotated image is h wide and w tall.
        if self.rot_deg == 90:      # raw -> rotated was (x, y) -> (h - 1 - y, x)
            return (y1, h - x2, y2, h - x1, score, cls)
        return (w - y2, x1, w - y1, x2, score, cls)

    def report(self):
        if not self.n_frames:
            self.get_logger().warn(
                "no frames processed in 10 s -- the RealSense driver only publishes "
                "images while a subscriber is attached, so check the image topic name")
            return
        ms = np.array(self.infer_ms)
        self.get_logger().info(
            f"{self.n_frames / 10.0:.1f} Hz processed ({self.n_skipped} frames skipped "
            f"to hold the rate), detect median {np.median(ms):.1f} ms, p95 "
            f"{np.percentile(ms, 95):.1f} ms")
        self.n_frames = self.n_skipped = 0
        self.infer_ms = []


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.detector.close()
    node.destroy_node()
    # ros2 run's signal handler may already have shut the context down; calling
    # again raises and makes a clean Ctrl-C look like a crash in the logs.
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
