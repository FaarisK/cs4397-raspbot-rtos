import time
import cv2

from vision import get_blue, get_green, get_red
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
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    blue_area, blue_mid, blue_bbox = get_blue(hsvImage)
    green_area, green_mid, green_bbox = get_green(hsvImage)
    red_area, red_mid, red_bbox = get_red(hsvImage)

    AREA_THRESHOLD = 10000

    bbox = None
    last_size = None

    if blue_area > AREA_THRESHOLD and blue_area > green_area and blue_area > red_area:
        last_color = 'blue'
        last_size = blue_area
        last_position = blue_mid
        bbox = blue_bbox
    elif green_area > AREA_THRESHOLD and green_area > blue_area and green_area > red_area:
        last_color = 'green'
        last_size = green_area
        last_position = green_mid
        bbox = green_bbox
    elif red_area > AREA_THRESHOLD and red_area > green_area and red_area > blue_area:
        last_color = 'red'
        last_size = red_area
        last_position = red_mid
        bbox = red_bbox
    else:
        last_color = None
        last_size = 0

    if bbox is not None:
        print(last_color, last_size, last_position)
        x1, y1, x2, y2 = bbox
        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        pass


def action_task(robot):
    if last_position is None:
        actions.IdleAction(robot)
        return

    # TODO uncomment
    if last_color == "green":
        robot.write_led_rgb_all(0, 255, 0)
        actions.GreenAction(robot, last_position)
    elif last_color == "blue":
        robot.write_led_rgb_all(0, 0, 255)
        actions.BlueAction(robot, last_position)
    elif last_color == "red":
        robot.write_led_rgb_all(255, 0, 0)
        actions.RedAction(robot)
    else:
        robot.write_led_rgb_all(0, 0, 0)
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

    while running and robot.get_angle() < 300 or robot.get_motor_state().va != 0:
        robot.update(0.005)
        time.sleep(0.005)

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
                    # log(f"[scheduler] running {task.name}")
                    a = time.monotonic()
                    task.run(now)
                    b = time.monotonic()
                    if b - a > task.period:
                        print("Task executed for longer than its period", task.name, format(task.period, '.4f'), format(b - a, '.4f'))

            time.sleep(0.005)

    except KeyboardInterrupt:
        log("[main] keyboard interrupt")

    finally:
        cap.release()
        robot.reset()
        log("[main] stopped safely")


if __name__ == "__main__":
    main()
