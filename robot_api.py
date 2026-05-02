import cv2
import time
from copy import copy

from Raspbot_Lib import Raspbot


class CameraAngle:
    def __init__(self, yaw, pitch):
        self.yaw = yaw
        self.pitch = pitch


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class MotorState:
    def __init__(self):
        self.fl = 0
        self.rl = 0
        self.fr = 0
        self.rr = 0
        self.vx = 0
        self.vy = 0
        self.va = 0


class Robot:
    MAX_SPEED = 100

    CAMERA_HOME = CameraAngle(90, 25)
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

    REG_MOTOR       = 0x01
    REG_SERVO       = 0x02
    REG_LED_INT_ALL = 0x03
    REG_LED_INT_ONE = 0x04
    REG_IR_SWITCH   = 0x05
    REG_BUZZER      = 0x06
    REG_UNKNOWN_07  = 0x07 # "Ctrl_Ulatist_Switch" or "Ctrl_getDis_Switch" Ultrasonic distance sensor?
    REG_LED_RGB_ALL = 0x08
    REG_LED_RGB_ONE = 0x09
    REG_IR_TRACK    = 0x0a
    # What about 0x0b?
    REG_UNKNOWN_0C  = 0x0c # Reading IR sensor data?
    # What about 0x0d-0x19?
    REG_UNKNOWN_1A  = 0x1a # Reading "Ulatist" "diss_L" (diss HIGH byte) Ultrasonic distance (millimeters)?
    REG_UNKNOWN_1B  = 0x1b # Reading "Ulatist" "diss_H" (diss LOW byte)  Ultrasonic distance (millimeters)?
    # Any more registers?
    # Potentiometers on the front control "distance from track sensors"
    # Can track data be measured or is it just a binary register?
    # (lights turn on when (variable distance) from floor).
    # Is ultrasonic a one-time pulse or a continuous measurement?
    # Bot can be remote controlled, is 0x05 and 0x0c for enabling and reading this?


    def __init__(self):
        self.__bot = Raspbot()
        self.reset()

    def reset(self):
        """
        Zeros the motors, homes the camera servos, disables lights and switches,
        and zeros the estimated and target states.
        """

        self.__bot.Ctrl_Muto(self.ID_MOTOR_FL, 0)
        self.__bot.Ctrl_Muto(self.ID_MOTOR_RL, 0)
        self.__bot.Ctrl_Muto(self.ID_MOTOR_FR, 0)
        self.__bot.Ctrl_Muto(self.ID_MOTOR_RR, 0)
        self.__bot.Ctrl_Servo(self.ID_SERVO_YAW, self.CAMERA_HOME.yaw)
        self.__bot.Ctrl_Servo(self.ID_SERVO_PITCH, self.CAMERA_HOME.pitch)
        self.__bot.Ctrl_WQ2812_ALL(False, 0)
        self.__bot.Ctrl_IR_Switch(False)
        self.__bot.Ctrl_BEEP_Switch(False)
        self.__bot.Ctrl_Ulatist_Switch(False)

        self.__motor_state = MotorState()
        self.__pos = Point(0, 0)
        self.__target_pos = Point(0, 0)
        self.__angle = 0
        self.__target_angle = 0
        self.__camera_angle = copy(self.CAMERA_HOME)
        self.__camera_target_angle = copy(self.CAMERA_HOME)
        self.__update_time = time.time()

    def turn_towards(self, angle):
        """
        Sets the bot's target angle within arbitrary bot-space.
        Units are degrees from the initial angle.

        Args:
            angle (float): New target angle. +A = Counter-clockwise
        """
        self.__target_angle = angle

    def get_angle(self):
        """
        Returns:
            float: Estimated angle relative to init. (+A = Counter-clockwise)
        """
        return self.__angle

    def move_towards(self, x, y):
        """
        Sets the bot's target position within arbitrary bot-space.
        Units are "distance traveled in 1 direction in a second."

        Args:
            x (float): X position component. (+X = Right relative to init)
            y (float): Y position component. (+Y = Forward relative to init)
        """
        self.__target_pos.x = x
        self.__target_pos.y = y

    def move_relative(self, dx, dy, dangle):
        """
        Sets the bot's target position relative to its current position within
        arbitrary bot-space. Calling this twice will *not* add the arguments.

        Args:
            dx (float): Lateral distance component. +X = Right
            dy (float): Longitudinal distance component. +Y = Forward
            dangle (float): New angle relative to current angle. +A = Counter-clockwise
        """

    def get_position(self):
        """
        Returns:
            Point: Gets the bot's current estimated position.
        """
        return copy(self.__pos)

    def home_camera_angle(self):
        """
        Sets the target angle of the camera servos to point straight forward.
        """
        self.__camera_target_angle.yaw = 90
        self.__camera_target_angle.pitch = 25

    def set_camera_angle(self, yaw, pitch):
        """
        Sets the target angle of the camera servos.
        Note that these cheap servos have *a lot* of backlash.
        Angles are very rough estimates at best and accuracy is both poor and
        variable.

        Args:
            yaw (float): Rotation around the vertical axis. 0 faces right, 180
            left.
            pitch (float): Rotation around the lateral axis. 0 faces down, 90
        """
        self.__camera_target_angle.yaw = yaw
        self.__camera_target_angle.pitch = pitch

    def get_camera_angle(self):
        """
        Returns:
            CameraAngle: Gets current camera angles.
            Note that these cheap servos have *a lot* of backlash.
            Angles are very rough estimates at best and accuracy is both poor
            and variable.
        """
        return copy(self.__camera_angle)

    def get_update_time(self):
        """
        Returns:
            float: System time (seconds) of last update call. Target adjustments
            and state interpolation of the next update are made relative to this
            time.
        """
        return self.__update_time

    def update(self):
        self.__lerp_state()
        # TODO determine motion vector (speed/angle/angular)
        # TODO translate to wheel and camera movements (based on update freq)
        # TODO update motor/servo parameters

    def __lerp_state(self):
        last_time = self.__update_time
        next_time = time.time()
        time_diff = next_time - last_time
        # TODO the rest of the math :p
