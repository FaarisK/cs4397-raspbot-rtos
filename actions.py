import sys
import time

LOOP_PERIOD = 0.10
TIMER_THRESHOLD = 30.0

CENTER_DEADZONE = 5.0
STRAFE_DEADZONE = 0.1

STRAFE_GAIN = 35.0

STARTUP_SPIN_DEG = 360.0
RED_TURN_DEG = 180.0


def TimerCheck(start_time):
    if time.monotonic() - start_time > TIMER_THRESHOLD:
        sys.exit(0)


def StartupAction(robot):
    robot.move_relative(0.0, 0.0, STARTUP_SPIN_DEG)


def IdleAction(robot):
    robot.move_relative(0.0, 0.0, 0.0)


def GreenAction(robot, position):
    angle_deg = (position - 320) / 640 * 60
    print(angle_deg)

    if abs(angle_deg) > CENTER_DEADZONE:
        robot.move_relative(0.0, 0.0, angle_deg)
    else:
        robot.move_relative(0.0, 0.0, 0.0)


def BlueAction(robot, position):
    cx_norm = (position - 320) / 640
    print(cx_norm * STRAFE_GAIN)

    if abs(cx_norm) > STRAFE_DEADZONE:
        robot.move_relative(cx_norm * STRAFE_GAIN, 0.0, 0.0)
    else:
        robot.move_relative(0.0, 0.0, 0.0)


def RedAction(robot):
    robot.move_relative(0.0, 0.0, RED_TURN_DEG)
