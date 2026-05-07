import time
import cv2

import vision
import actions
from robot_api import Robot


DEBUG = True

RUN_TIME = 30.0

ROBOT_UPDATE_PERIOD = 0.10
VISION_PERIOD = 0.15
ACTION_PERIOD = 0.10
SAFETY_PERIOD = 0.10

last_color = None
last_position = None
start_time = None
running = True


class Task:
    def __init__(self, name, period, callback):
        self.name = name
        self.period = period
        self.callback = callback
        self.next_run = time.monotonic()

    def due(self, now):
        return now >= self.next_run

    def run(self, now):
        self.callback()
        self.next_run = now + self.period


def log(message):
    if DEBUG:
        print(message)


def safety_task():
    global running

    elapsed = time.monotonic() - start_time

    if elapsed >= RUN_TIME:
        log("[safety] timer finished")
        running = False


def vision_task(cap):
    global last_color
    global last_position

    ret, frame = cap.read()

    if not ret:
        log("[vision] camera frame not read")
        last_color = None
        last_position = None
        return

    hsv = vision.preprocess(frame)
    color_counts = vision.ColorCounter(hsv)
    detected_color, position = vision.ColorLocator(hsv, color_counts)

    last_color = detected_color
    last_position = position

    log(f"[vision] color={last_color}, pos={last_position}")


def action_task(robot):
    if last_position is None:
        actions.IdleAction(robot)
        return

    if last_color == "green":
        actions.GreenAction(robot, last_position)
    elif last_color == "blue":
        actions.BlueAction(robot, last_position)
    elif last_color == "red":
        actions.RedAction(robot)
    else:
        actions.IdleAction(robot)


def robot_update_task(robot):
    robot.update(ROBOT_UPDATE_PERIOD)


def main():
    global start_time
    global running

    robot = Robot()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 15)

    start_time = time.monotonic()
    running = True

    log("[startup] running startup action")
    actions.StartupAction(robot)
    log("[startup] entering round-robin scheduler")

    tasks = [
        Task("safety", SAFETY_PERIOD, safety_task),
        Task("vision", VISION_PERIOD, lambda: vision_task(cap)),
        Task("action", ACTION_PERIOD, lambda: action_task(robot)),
        Task("robot_update", ROBOT_UPDATE_PERIOD, lambda: robot_update_task(robot)),
    ]

    try:
        while running:
            now = time.monotonic()

            for task in tasks:
                if task.due(now):
                    log(f"[scheduler] running {task.name}")
                    task.run(now)

            time.sleep(0.005)

    except KeyboardInterrupt:
        log("[main] keyboard interrupt")

    finally:
        cap.release()
        robot.reset()
        log("[main] stopped safely")


if __name__ == "__main__":
    main()
