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


ACTIONS.PY

actions.py defines the robot behaviors triggered by each detected color and the
supporting utilities for timing and idle state.

StartupAction(robot)
    Spins the robot 360 degrees on startup as a boot signal, then resets state.
    Runs for about 1.8 seconds before returning.

TimerCheck(start_time)
    Exits the program if the total elapsed time since start_time exceeds
    TIMER_THRESHOLD (30 seconds).

IdleAction(robot)
    Sends a zero-movement command and ticks the robot update. Called when no
    color is detected.

GreenAction(robot, p_vector)
    Rotates the robot toward the detected green object using the horizontal
    angle from vision.py. Stops rotating once within CENTER_DEADZONE (5 deg).
    ex: angle_deg=10.5 -> rotate right; angle_deg=2.0 -> stop (in deadzone)

BlueAction(robot, p_vector)
    Strafes the robot left or right to center the detected blue object
    horizontally. Stops strafing once within STRAFE_DEADZONE (0.1 cx_norm).
    ex: cx_norm=0.4 -> strafe right; cx_norm=0.05 -> stop (in deadzone)

RedAction(robot)
    Spins the robot 180 degrees when red is detected, then resets state.
    Runs for about 0.9 seconds before returning.
