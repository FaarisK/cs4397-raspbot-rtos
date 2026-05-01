import cv2
from time import time
from copy import copy

from Raspbot_Lib import Raspbot


class Angle:
    def __init__(self, yaw, pitch):
        self.yaw = yaw
        self.pitch = pitch


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Size:
    def __init__(self, w, h):
        self.w = w
        self.h = h


class Robot:
    CAMERA_HOME = Angle(90, 25)
    CAMERA_YAW_MIN = 0
    CAMERA_YAW_MAX = 180
    CAMERA_PITCH_MIN = 0
    CAMERA_PITCH_MAX = 115
    ID_MOTOR_FL = 0
    ID_MOTOR_RL = 1
    ID_MOTOR_FR = 2
    ID_MOTOR_RR = 3
    ID_SERVO_YAW = 1
    ID_SERVO_PITCH = 2

    def __init__(self):
        self.__bot = Raspbot()
        self.reset_motion()

    def reset_motion(self):
        self.__pos = Point(0, 0)
        self.__target_pos = Point(0, 0)
        self.__angle = 0
        self.__target_angle = 0
        self.__camera_angle = copy(self.CAMERA_HOME)
        self.__camera_target_angle = copy(self.CAMERA_HOME)
        self.__update_time = time.time()

    def turn_towards(self, angle):
        self.__target_angle = angle

    def get_angle(self):
        return self.__angle

    def move_towards(self, x, y):
        self.__target_pos.x = x
        self.__target_pos.y = y

    def get_position(self):
        return copy(self.__pos)

    def home_camera_angle(self):
        self.__camera_target_angle.yaw = 90
        self.__camera_target_angle.pitch = 25

    def set_camera_angle(self, yaw, pitch):
        self.__camera_target_angle.yaw = yaw
        self.__camera_target_angle.pitch = pitch

    def get_camera_angle(self):
        return copy(self.__camera_angle)

    def get_update_time(self):
        return self.__update_time

    def stop(self, reset=True):
        self.__bot.Ctrl_Muto(self.ID_MOTOR_FL, 0)
        self.__bot.Ctrl_Muto(self.ID_MOTOR_RL, 0)
        self.__bot.Ctrl_Muto(self.ID_MOTOR_FR, 0)
        self.__bot.Ctrl_Muto(self.ID_MOTOR_RR, 0)
        self.__bot.Ctrl_Servo(self.ID_SERVO_YAW, self.CAMERA_HOME.yaw)
        self.__bot.Ctrl_Servo(self.ID_SERVO_PITCH, self.CAMERA_HOME.pitch)
        if reset:
            self.reset_motion()

    def update(self):
        last_time = self.__update_time
        next_time = time.time()
        time_diff = next_time - last_time
        # TODO determine motion vector (speed/angle/angular)
        # TODO translate to wheel and camera movements (based on update freq)
        # TODO update motor/servo parameters
