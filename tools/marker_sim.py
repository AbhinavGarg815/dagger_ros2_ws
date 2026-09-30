#!/usr/bin/env python3
"""Rendered ArUco frames for the marker tests -- the rover's real camera. No ROS.

Markers are drawn into 640x480 frames through the D435 model and upside-down
mount of this rover (K and base->optical TF from the reid3 recording,
2026-09-30), camera-like: 4x supersampled, area-averaged, lens blur, noise.
Used by tools/test_marker_tracking.py and tools/test_marker_pipeline.py.
"""
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "dagger_bringup"))
from dagger_bringup.marker_tracking import quat_to_R  # noqa: E402

# the rover's camera (reid3 camera_info + tf_static)
K = (606.0390625, 605.5621948242188, 329.91619873046875, 259.3073425292969)
DIST = [0.0] * 5
CAM_T = np.array([0.29250786138934926, -0.01468633208480806, 0.9898413473458343])
CAM_Q = (0.5004584200522816, 0.4961645609855828, 0.5016558184696026, 0.5017005772930675)   # x, y, z, w
CAM_R = quat_to_R(CAM_Q)
W, H = 640, 480
DICT = "DICT_4X4_50"
SS = 4                                            # supersampling of the render


def project(P, s=1.0):
    """base_footprint points (N, 3) -> pixels (N, 2) as the driver delivers them
    (s > 1: in an s-times supersampled image)."""
    o = (P - CAM_T) @ CAM_R                       # R^T (P - t), row-wise
    uv = np.c_[K[0] * o[:, 0] / o[:, 2] + K[2], K[1] * o[:, 1] / o[:, 2] + K[3]]
    return (uv + 0.5) * s - 0.5


def render(markers, seed=0, blur=0.7, noise=3.0, dictionary=DICT):
    """markers: [(marker_id, size m, centre (x, y, z) base, normal yaw deg)] ->
    BGR frame. The normal points out of the printed face; yaw 180 = facing the
    rover. Each marker has a one-cell white margin (quiet zone), as printed.
    Camera-like: drawn 4x supersampled, area-averaged down, lens blur, noise."""
    rng = np.random.default_rng(seed)
    bg = cv2.GaussianBlur((rng.random((H, W)) * 200 + 30).astype(np.uint8), (0, 0), 2.5)
    img = cv2.resize(bg, (W * SS, H * SS), interpolation=cv2.INTER_LINEAR)
    A = cv2.aruco
    d = A.getPredefinedDictionary(getattr(A, dictionary))
    px = (int(d.markerSize) + 2) * 40              # 40 px per cell, black border included
    for mid, size, c, yaw in markers:
        m = A.drawMarker(d, mid, px)
        board = cv2.copyMakeBorder(m, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
        n = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0.0])
        up = np.array([0.0, 0.0, 1.0])
        right = np.cross(-n, up)                   # as seen by someone facing the print
        h = size / 2
        c = np.asarray(c, float)
        P = np.array([c - right * h + up * h, c + right * h + up * h,
                      c + right * h - up * h, c - right * h - up * h])
        dst = project(P, SS).astype(np.float32)
        e = 39.5 + px
        src = np.float32([[39.5, 39.5], [e, 39.5], [e, e], [39.5, e]])
        Hm = cv2.getPerspectiveTransform(src, dst)
        size_ss = (W * SS, H * SS)
        warped = cv2.warpPerspective(board, Hm, size_ss, flags=cv2.INTER_NEAREST)
        mask = cv2.warpPerspective(np.full((px + 80, px + 80), 255, np.uint8), Hm, size_ss,
                                   flags=cv2.INTER_NEAREST) > 0
        img[mask] = warped[mask]
    img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
    img = cv2.GaussianBlur(img, (0, 0), blur).astype(np.float32) + rng.normal(0, noise, (H, W))
    return cv2.cvtColor(np.clip(img, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
