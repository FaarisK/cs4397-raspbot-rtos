CS 4397
Nishant Bhagat


VISION.PY

vision.py does the color detection for the robot using the camera. It figures
out what color is being held in front of it and where that color is in the frame.

ColorCounter(hsv)
    Counts how many pixels match each command color (green, blue, red) and
    returns a dictionary of the counts. Red uses two HSV ranges since it wraps
    around the hue wheel.
    ex: {"green": 46000, "blue": 0, "red": 1050}

ColorLocator(hsv, color_counts)
    Takes the counts from ColorCounter and finds the dominant color. If enough
    pixels are detected (at least 500), it returns the average position of those
    pixels normalized from -1 to 1, plus the horizontal angle based on the
    camera's 60 degree FOV. Returns None if nothing is detected.
    ex: ("green", (0.35, 0.12, 10.5))  -> (cx_norm, cy_norm, angle_deg)
    ex: ("green", None)                 -> nothing detected
