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
import array
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CompressedImage
from dagger_msgs.msg import (MarkerDetection, MarkerDetections, PersonDetection,
                             PersonDetections)
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
        # JPEG of every PROCESSED frame for the click-to-follow browser UI
        # (ui_server). Same header as that frame's detections, so the boxes the
        # operator clicks always belong to the image they see. Native orientation
        # (the browser rotates). Encoded only while someone subscribes.
        self.declare_parameter("ui_image_topic", "/follow/ui/image/compressed")
        self.declare_parameter("ui_jpeg_quality", 70)
        # Re-ID appearance signature per person (OSNet TensorRT, reid_trt.py), from
        # the UPRIGHT frame used for inference, on /follow/person_detections
        # (dagger_msgs/PersonDetections) for person_tracker. Empty = no signatures.
        self.declare_parameter("reid_engine", "")
        self.declare_parameter("person_detections_topic", "/follow/person_detections")
        self.declare_parameter("reid_min_score", 0.3)
        # ArUco markers on the SAME frame (marker_tracking.aruco_detector, ~6 ms on
        # the CPU), on /follow/marker_detections for marker_tracker. "" = off.
        # The dictionary must be the one the marker was printed from
        # (tools/marker_probe.py tells). Detected on the frame as the driver
        # delivers it: ArUco does not care about rotation.
        self.declare_parameter("marker_dictionary", "")
        self.declare_parameter("markers_topic", "/follow/marker_detections")
        # The ids being looked for ("7" or "3,7"; "" = any): the detector's second
        # pass runs only while one of them is missing (marker_tracker has the same
        # list and does the filtering).
        self.declare_parameter("marker_ids", "", ParameterDescriptor(dynamic_typing=True))
        # False = markers only: YOLOX is not loaded and no people are reported
        # (empty detections are still published, so the UI keeps its frames).
        self.declare_parameter("detect_people", True)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        engine = g("engine_path")
        self.detect_people = bool(g("detect_people"))
        if self.detect_people and not engine:
            raise RuntimeError(
                "engine_path is required -- build one with tools/fetch_model.sh")
        self.period = 1.0 / float(g("target_rate"))
        self.publish_debug = bool(g("publish_debug_image"))
        self.rot_deg = int(g("image_rotation_deg")) % 360
        self.ui_quality = int(g("ui_jpeg_quality"))
        self.reid_min_score = float(g("reid_min_score"))
        self.reid = None
        self.marker_dict, self.marker_detect, self.marker_ms = g("marker_dictionary"), None, []
        if self.marker_dict:
            try:
                from dagger_bringup.marker_tracking import aruco_detector
                ids = [int(i) for i in str(g("marker_ids")).replace(";", ",").split(",") if i.strip()]
                self.marker_detect = aruco_detector(self.marker_dict, want_ids=ids or None)
            except Exception as e:     # a bad dictionary name must not take detection down
                self.get_logger().error(f"marker detection disabled: {e}")
        if g("reid_engine") and self.detect_people:
            try:
                from dagger_bringup.reid_trt import ReidTRT
                self.reid = ReidTRT(g("reid_engine"))
            except Exception as e:     # a missing engine must not take detection down with it
                self.get_logger().error(f"re-ID disabled -- cannot load {g('reid_engine')}: {e} "
                                        "(build it: bash tools/fetch_model.sh reid)")
        self.reid_ms = []
        if self.rot_deg not in (0, 90, 180, 270):
            raise RuntimeError(f"image_rotation_deg must be 0/90/180/270, got {self.rot_deg}")

        self.detector = YoloxTRT(engine,
                                 conf_thres=float(g("conf_threshold")),
                                 nms_thres=float(g("nms_threshold")),
                                 classes=(COCO_PERSON,)) if self.detect_people else None

        from cv_bridge import CvBridge
        self.bridge = CvBridge()
        from concurrent.futures import ThreadPoolExecutor
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="side")

        # Depth 1 + best effort: we want the newest frame, never a queue. A
        # backlog here would feed the controller a stale bearing, which is worse
        # than skipping a frame.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Detection2DArray, g("detections_topic"), 10)
        self.debug_pub = (self.create_publisher(Image, g("debug_image_topic"), 1)
                          if self.publish_debug else None)
        self.ui_pub = self.create_publisher(CompressedImage, g("ui_image_topic"), qos)
        self.pd_pub = self.create_publisher(PersonDetections, g("person_detections_topic"), 10)
        self.mk_pub = self.create_publisher(MarkerDetections, g("markers_topic"), 10)
        self.create_subscription(Image, g("image_topic"), self.on_image, qos)

        self.last_run = 0.0
        self.n_frames = 0
        self.n_skipped = 0
        self.infer_ms = []
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"person_detector: {g('image_topic')} -> {g('detections_topic')} at "
            f"{g('target_rate')} Hz, "
            + (f"conf {g('conf_threshold')}, engine {engine.split('/')[-1]}, input "
               f"{self.detector.input_hw}" if self.detect_people else "people OFF (markers only)")
            + (f"; ArUco {self.marker_dict} -> {g('markers_topic')}" if self.marker_detect else ""))

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
        # VIDEO FIRST: the operator's JPEG and the ArUco markers, in a worker
        # thread WHILE YOLOX runs (cv2 releases the GIL; measured 40.6 -> 28.2 ms
        # for both together, 2026-10-02). The JPEG used to wait for YOLOX + markers:
        # 99 ms from capture to publish, of which 55 ms was detection it does not
        # need -- the page now gets the frame first and its boxes when they exist.
        side = self.pool.submit(self.side_work, frame, msg.header)
        t0 = time.perf_counter()
        upright = self.rotate(frame)
        dets_up = self.detector.detect(upright) if self.detector is not None else []
        dets = [self.unrotate(d, frame.shape[1], frame.shape[0]) for d in dets_up]
        self.infer_ms.append((time.perf_counter() - t0) * 1e3)
        self.n_frames += 1

        # appearance signatures: crops from the upright frame (the network was
        # trained on upright people), in the same order as dets
        embs = [None] * len(dets)
        if self.reid is not None and self.pd_pub.get_subscription_count() > 0:
            from dagger_bringup.reid_trt import crop
            t1 = time.perf_counter()
            idx, crops = [], []
            for k, (x1, y1, x2, y2, score, _c) in enumerate(dets_up[:16]):
                c = crop(upright, (x1, y1, x2, y2)) if score >= self.reid_min_score else None
                if c is not None:
                    idx.append(k)
                    crops.append(c)
            for k, e in zip(idx, self.reid.embed(crops)):
                embs[k] = e
            self.reid_ms.append((time.perf_counter() - t1) * 1e3)
        pd = PersonDetections()
        pd.header = msg.header
        pd.image_width, pd.image_height = frame.shape[1], frame.shape[0]
        for (x1, y1, x2, y2, score, _c), e in zip(dets, embs):
            p = PersonDetection()
            p.x1, p.y1, p.x2, p.y2, p.score = float(x1), float(y1), float(x2), float(y2), float(score)
            if e is not None:
                p.embedding = array.array("f", e.astype(np.float32).tobytes())
            pd.detections.append(p)
        self.pd_pub.publish(pd)

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
        side.result()                      # done long before YOLOX; and never draw on `frame` before it is

        if self.debug_pub is not None:
            import cv2
            for x1, y1, x2, y2, score, _c in dets:
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"{score:.2f}", (int(x1), max(12, int(y1) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            dbg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            dbg.header = msg.header
            self.debug_pub.publish(dbg)

    def side_work(self, frame, header):
        """Worker thread, concurrent with YOLOX: the UI JPEG first, then markers.
        Reads `frame` only. Never raises: a failure here must not stop detection."""
        import cv2
        try:
            if self.ui_pub.get_subscription_count() > 0:
                ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.ui_quality])
                if ok:
                    ui = CompressedImage()
                    ui.header = header
                    ui.format = "jpeg"
                    # array('B'), NOT bytes: rclpy validates a bytes value element by
                    # element in Python -- 32 ms for a 166 KB JPEG vs 0.11 ms (measured)
                    ui.data = array.array("B", jpg.tobytes())
                    self.ui_pub.publish(ui)
        except Exception as e:
            self.get_logger().error(f"UI image failed: {e}", throttle_duration_sec=5.0)
        if self.marker_detect is None:
            return
        t1 = time.perf_counter()
        mk = MarkerDetections()
        mk.header = header
        mk.image_width, mk.image_height = frame.shape[1], frame.shape[0]
        mk.dictionary = self.marker_dict
        try:
            for mid, corners in self.marker_detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)):
                m = MarkerDetection()
                m.marker_id, m.corners = mid, corners
                mk.markers.append(m)
        except Exception as e:             # never let a marker problem stop person detection
            self.get_logger().error(f"marker detection failed: {e}", throttle_duration_sec=5.0)
        self.mk_pub.publish(mk)            # every frame, also empty: marker_tracker's clock
        self.marker_ms.append((time.perf_counter() - t1) * 1e3)

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
            f"{np.percentile(ms, 95):.1f} ms"
            + (f", re-ID median {np.median(self.reid_ms):.1f} ms" if self.reid_ms else "")
            + (f", markers median {np.median(self.marker_ms):.1f} ms" if self.marker_ms else ""))
        self.n_frames = self.n_skipped = 0
        self.infer_ms = []
        self.reid_ms = []
        self.marker_ms = []


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pool.shutdown(wait=True)
    if node.detector is not None:
        node.detector.close()
    node.destroy_node()
    # ros2 run's signal handler may already have shut the context down; calling
    # again raises and makes a clean Ctrl-C look like a crash in the logs.
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
