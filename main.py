# main.py
# Main loop + scheduler for CS4397 Raspbot project
#
# This file connects:
#   1. imageget.py  -> gets camera image as HSV
#   2. vision.py    -> counts/locates dominant color
#   3. robot_api.py -> moves/turns the robot
#
# Knight / Faaris contribution:
#   - stable scheduled loop
#   - vision-to-movement behavior logic
#   - safe shutdown/reset

import time

import imageget
import vision
from robot_api import Robot


# Scheduler / timing constants
LOOP_PERIOD = 0.10          # seconds between control updates, 10 Hz
RUN_TIME = 30.0             # total demo runtime in seconds

# Behavior tuning constants
CENTER_DEADZONE_DEG = 7.0   # if target angle is within this, treat as centered
TURN_GAIN = 0.8             # scales vision angle into robot turn amount
SEARCH_TURN_STEP = 12.0     # degrees to rotate while searching
FORWARD_STEP = 0.20         # arbitrary distance units forward per movement decision

DEBUG = True


def log(message):
    if DEBUG:
        print(message)


def choose_action(robot, detected_color, position):
    """
    Decides what the robot should do based on color detection.

    Args:
        robot (Robot): robot API object
        detected_color (str): dominant color found by vision.py
        position:
            None if no valid object found,
            otherwise (cx_norm, cy_norm, angle_deg)
    """

    # No strong color target found, slowly rotate to search.
    if position is None:
        robot.move_relative(0.0, 0.0, SEARCH_TURN_STEP)
        log(f"[search] no target found, rotating {SEARCH_TURN_STEP:.2f} deg")
        return

    cx_norm, cy_norm, angle_deg = position

    log(
        f"[vision] color={detected_color}, "
        f"cx={cx_norm:.2f}, cy={cy_norm:.2f}, angle={angle_deg:.2f}"
    )

    # If the object is not centered, rotate toward it.
    if abs(angle_deg) > CENTER_DEADZONE_DEG:
        correction = angle_deg * TURN_GAIN

        robot.move_relative(0.0, 0.0, correction)

        log(
            f"[turn] target off-center, "
            f"correction={correction:.2f} deg"
        )
        return

    # If target is centered, move forward.
    robot.move_relative(0.0, FORWARD_STEP, 0.0)
    log(f"[move] target centered, moving forward {FORWARD_STEP} units")


def main():
    robot = Robot()

    start_time = time.time()
    next_tick = start_time

    log("[main] starting robot scheduler")
    log(f"[main] running for {RUN_TIME} seconds")

    try:
        while True:
            now = time.time()
            elapsed = now - start_time

            if elapsed >= RUN_TIME:
                log("[main] demo timer finished")
                break

            # Keep loop timing stable.
            if now < next_tick:
                time.sleep(next_tick - now)

            tick_start = time.time()

            # 1. Get HSV image from camera.
            hsv = imageget.gethsv()

            # 2. Count colors from HSV image.
            color_counts = vision.ColorCounter(hsv)

            # 3. Locate dominant color.
            detected_color, position = vision.ColorLocator(hsv, color_counts)

            # 4. Choose search/turn/forward behavior.
            choose_action(robot, detected_color, position)

            # 5. Apply movement update through robot_api.
            robot.update(LOOP_PERIOD)

            # 6. Schedule next tick.
            next_tick = tick_start + LOOP_PERIOD

    except KeyboardInterrupt:
        log("\n[main] keyboard interrupt received")

    except Exception as error:
        log(f"\n[main] error: {error}")

    finally:
        log("[main] resetting robot and stopping motors")
        robot.reset()
        log("[main] program ended safely")


if __name__ == "__main__":
    main()
