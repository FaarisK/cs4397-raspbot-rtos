# NOTE: Camera can safely capture at 15 FPS (0.066~ seconds/frame) with sub-
#       millisecond latency per frame. Capturing at higher framerates (e.g. 20
#       FPS) *WILL* result in blocking while the camera buffers the next frame.

# NOTE: Just looping over all pixels can take 100-200ms, an optimized image scan
#       is needed. Multi-threaded/multi-process image processing may not be a
#       terrible idea (barring deadline constraints).

# NOTE: Horizontal FoV is ~60 degrees

import cv2
import numpy as np

IDLE_POSITION = None
MIN_PIXEL_THRESHOLD = 500
SCALE = 0.5
HALF_HFOV_DEG = 30.0

COLOR_RANGES = {
    "green": [(35, 50, 50),  (85, 255, 255)],
    "blue":  [(100, 80, 50), (130, 255, 255)],
    "red":   [[(0, 100, 50),   (10, 255, 255)],
              [(170, 100, 50), (180, 255, 255)]],
}

def _make_mask(hsv, color):
    ranges = COLOR_RANGES[color]
    if color == "red":
        m1 = cv2.inRange(hsv, np.array(ranges[0][0]), np.array(ranges[0][1]))
        m2 = cv2.inRange(hsv, np.array(ranges[1][0]), np.array(ranges[1][1]))
        return cv2.bitwise_or(m1, m2)
    return cv2.inRange(hsv, np.array(ranges[0]), np.array(ranges[1]))


def preprocess(frame):
    small = cv2.resize(frame, (0, 0), fx=SCALE, fy=SCALE)
    return cv2.cvtColor(small, cv2.COLOR_BGR2HSV)


def ColorCounter(hsv):
    return {color: int(cv2.countNonZero(_make_mask(hsv, color)))
            for color in COLOR_RANGES}


def ColorLocator(hsv, color_counts):
    dominant = max(color_counts, key=color_counts.get)

    if color_counts[dominant] < MIN_PIXEL_THRESHOLD:
        return dominant, IDLE_POSITION

    mask = _make_mask(hsv, dominant)
    pts = cv2.findNonZero(mask)

    if pts is None:
        return dominant, IDLE_POSITION

    cx_px = float(np.mean(pts[:, 0, 0]))
    cy_px = float(np.mean(pts[:, 0, 1]))

    w = hsv.shape[1]
    h = hsv.shape[0]
    cx_norm = (cx_px - w / 2.0) / (w / 2.0)   
    cy_norm = (cy_px - h / 2.0) / (h / 2.0)   
    angle_deg = cx_norm * HALF_HFOV_DEG

    return dominant, (cx_norm, cy_norm, angle_deg)