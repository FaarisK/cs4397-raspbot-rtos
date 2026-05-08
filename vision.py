# NOTE: Camera can safely capture at 15 FPS (0.066~ seconds/frame) with sub-
#       millisecond latency per frame. Capturing at higher framerates (e.g. 20
#       FPS) *WILL* result in blocking while the camera buffers the next frame.

# NOTE: Just looping over all pixels can take 100-200ms, an optimized image scan
#       is needed. Multi-threaded/multi-process image processing may not be a
#       terrible idea (barring deadline constraints).

# NOTE: Horizontal FoV is ~60 degrees

import numpy as np
import cv2
from PIL import Image

# color BGR
blue = [172, 48, 40]
green = [20, 255, 19]
red = [0, 0, 255]


def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue wrap-around
    if hue >= 170:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 0, 192, 128], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 128, 128], dtype=np.uint8)
        upperLimit = np.array([hue + 0, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 5, 192, 128], dtype=np.uint8)
        upperLimit = np.array([hue + 5, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit


def detect_colors(hsvImage, lowerLimit, upperLimit):
    mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
    mask = Image.fromarray(mask)
    bbox = mask.getbbox()
    if bbox is None:
        return 0, 0, None
    x1, y1, x2, y2 = bbox

    return (x2 - x1) * (y2 - y1), (x1 + x2) / 2, bbox


def get_blue(hsvImage):
    lowerLimit, upperLimit = get_limits(color=blue)
    return detect_colors(hsvImage, lowerLimit, upperLimit)


def get_green(hsvImage):
    lowerLimit, upperLimit = get_limits(color=green)
    return detect_colors(hsvImage, lowerLimit, upperLimit)


def get_red(hsvImage):
    lowerLimit, upperLimit = get_limits(color=red)
    return detect_colors(hsvImage, lowerLimit, upperLimit)
