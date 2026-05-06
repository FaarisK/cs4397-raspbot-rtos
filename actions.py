import time
import sys

LOOP_PERIOD       = 0.10   # seconds per tick
TIMER_THRESHOLD   = 30.0   # seconds before program exits
CENTER_DEADZONE   = 5.0    # degrees - GreenAction stops rotating within this
STRAFE_DEADZONE   = 0.1    # cx_norm - BlueAction stops strafing within this
TURN_GAIN         = 0.8    # scales angle_deg to rotation amount per tick
STRAFE_GAIN       = 1.0    # scales cx_norm to lateral displacement per tick


def StartupAction(robot):
    robot.move_relative(0, 0, 360)
    start = time.time()
    while time.time() - start < 1.8:
        robot.update(LOOP_PERIOD)
        time.sleep(LOOP_PERIOD)
    robot.reset()


def TimerCheck(start_time):
    if time.time() - start_time > TIMER_THRESHOLD:
        sys.exit(0)


def IdleAction(robot):
    robot.move_relative(0, 0, 0)
    robot.update(LOOP_PERIOD)


def GreenAction(robot, p_vector):
    cx_norm, cy_norm, angle_deg = p_vector
    if abs(angle_deg) > CENTER_DEADZONE:
        robot.move_relative(0, 0, angle_deg * TURN_GAIN)
    else:
        robot.move_relative(0, 0, 0)
    robot.update(LOOP_PERIOD)


def BlueAction(robot, p_vector):
    cx_norm, cy_norm, angle_deg = p_vector
    if abs(cx_norm) > STRAFE_DEADZONE:
        robot.move_relative(cx_norm * STRAFE_GAIN, 0, 0)
    else:
        robot.move_relative(0, 0, 0)
    robot.update(LOOP_PERIOD)


def RedAction(robot):
    robot.move_relative(0, 0, 180)
    start = time.time()
    while time.time() - start < 0.9:
        robot.update(LOOP_PERIOD)
        time.sleep(LOOP_PERIOD)
    robot.reset()
