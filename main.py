import time
import cv2

import vision
import actions
from robot_api import Robot

DEBUG = True


def log(message):
    if DEBUG:
        print(message)


def main():
    robot = Robot()
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 15)

    start_time = time.time()

    log("[startup] spinning 360 degrees")
    actions.StartupAction(robot)
    log("[startup] done, entering round-robin loop")

    try:
        while True:
            # Task 1: TimerCheck
            actions.TimerCheck(start_time)

            # Task 2: Capture frame
            ret, frame = cap.read()
            if not ret:
                continue

            # Task 3: ColorCounter
            hsv = vision.preprocess(frame)
            color_counts = vision.ColorCounter(hsv)

            # Task 4: ColorLocator
            detected_color, position = vision.ColorLocator(hsv, color_counts)

            log(f"[vision] color={detected_color}, pos={position}")

            # Task 5: Dispatch action based on detected color
            if position is None:
                actions.IdleAction(robot)
            elif detected_color == "green":
                actions.GreenAction(robot, position)
            elif detected_color == "blue":
                actions.BlueAction(robot, position)
            elif detected_color == "red":
                actions.RedAction(robot)

            time.sleep(actions.LOOP_PERIOD)

    except KeyboardInterrupt:
        log("[main] keyboard interrupt")

    finally:
        cap.release()
        robot.reset()
        log("[main] stopped")


if __name__ == "__main__":
    main()
