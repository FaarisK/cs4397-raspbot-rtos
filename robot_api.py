import cv2
import math
import time
from copy import copy

from Raspbot_Lib import Raspbot


class CameraAngle:
    def __init__(self, yaw: int, pitch: int):
        self.yaw = yaw
        self.pitch = pitch

    def __str__(self):
        return f'({self.yaw}, {self.pitch})'


class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __str__(self):
        x = format(self.x, '.3f')
        y = format(self.y, '.3f')
        return f'({x}, {y})'


class MotorState:
    def __init__(self):
        self.fl = 0
        self.rl = 0
        self.fr = 0
        self.rr = 0
        self.vx = 0.0
        self.vy = 0.0
        self.va = 0.0

    def __str__(self):
        fl = format(self.fl, '03')
        rl = format(self.rl, '03')
        fr = format(self.fr, '03')
        rr = format(self.rr, '03')
        vx = format(self.vx, '.3f')
        vy = format(self.vy, '.3f')
        va = format(self.va, '.3f')
        return f'| {fl}  {fr} | {vx}  {vy}\n| {rl}  {rr} | {va}'


class Robot:
    MAX_POWER = 100  # Up to 255 - Too high will shut off Pi
    # distance/(sec*power) (completely arbitrary units)
    MOVEMENT_RATE = 1.0
    # 100 power for 1.8s = ~360deg
    # 360/1.8/100 = 2 deg/(sec*power)
    ROTATION_RATE = math.radians(2.25)
    MIN_POWER = 0
    ROOT2 = math.sqrt(2.0)
    # How far out of phase is wheel rotation from motion vector
    WHEEL_PHASE = math.pi * 0.25

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

    REG_MOTOR = 0x01
    REG_SERVO = 0x02
    REG_LED_INT_ALL = 0x03
    REG_LED_INT_ONE = 0x04
    REG_IR_SWITCH = 0x05
    REG_BUZZER = 0x06
    REG_UNKNOWN_07 = 0x07  # "Ctrl_Ulatist_Switch" or "Ctrl_getDis_Switch" Ultrasonic distance sensor?
    REG_LED_RGB_ALL = 0x08
    REG_LED_RGB_ONE = 0x09
    REG_IR_TRACK = 0x0A
    # What about 0x0b?
    REG_UNKNOWN_0C = 0x0C  # Reading IR sensor data?
    # What about 0x0d-0x19?
    REG_UNKNOWN_1A = 0x1A  # Reading "Ulatist" "diss_L" (diss HIGH byte) Ultrasonic distance (millimeters)?
    REG_UNKNOWN_1B = 0x1B  # Reading "Ulatist" "diss_H" (diss LOW byte)  Ultrasonic distance (millimeters)?
    # Any more registers?
    # Potentiometers on the front control "distance from track sensors"
    # Can track data be measured or is it just a binary register?
    # (lights turn on when (variable distance) from floor).
    # Is ultrasonic a one-time pulse or a continuous measurement?
    # Bot can be remote controlled, is 0x05 and 0x0c for enabling and reading this?

    def set_rate_calibration(self, movement=1.0, rotation=2.25):
        """
        Sets the "constant" values for movement rates. This calibration could
        be estimated after a short test run or measured using the camera.

        Args:
            rotation (float, optional): The rotation rate in [degrees per
                                        second per unit of power].
                                        Defaults to 2.4.
            movement (float, optional): The movement rate in [distance per
                                        second per unit of power].
                                        Defaults to 1.0.
        """
        Robot.MOVEMENT_RATE = movement
        Robot.ROTATION_RATE = math.radians(rotation)

    def __init__(self):
        self.__bot = Raspbot()
        self.reset()

    def reset(self) -> None:
        """
        Zeros the motors, homes the camera servos, disables lights and
        switches, and zeros the estimated and target states.
        """
        self.__bot.Ctrl_Muto(Robot.ID_MOTOR_FL, 0)
        self.__bot.Ctrl_Muto(Robot.ID_MOTOR_RL, 0)
        self.__bot.Ctrl_Muto(Robot.ID_MOTOR_FR, 0)
        self.__bot.Ctrl_Muto(Robot.ID_MOTOR_RR, 0)
        self.__bot.Ctrl_Servo(Robot.ID_SERVO_YAW, Robot.CAMERA_HOME.yaw)
        self.__bot.Ctrl_Servo(Robot.ID_SERVO_PITCH, Robot.CAMERA_HOME.pitch)
        self.__bot.Ctrl_WQ2812_ALL(False, 0)
        self.__bot.Ctrl_IR_Switch(False)
        self.__bot.Ctrl_BEEP_Switch(False)
        self.__bot.Ctrl_Ulatist_Switch(False)

        self.__motor_state = MotorState()
        self.__pos = Point(0.0, 0.0)
        self.__target_pos = Point(0.0, 0.0)
        self.__angle = 0.0
        self.__target_angle = 0.0
        self.__camera_angle = copy(Robot.CAMERA_HOME)
        self.__camera_target_angle = copy(Robot.CAMERA_HOME)
        self.__update_time = time.time()

    def get_motor_state(self):
        """
        Returns:
            MotorState: A copy of the motor state after the last update() call.
        """
        return copy(self.__motor_state)

    def turn_towards(self, angle: float) -> None:
        """
        Sets the bot's target angle.

        Args:
            angle (float): New target angle (degrees) relative to init.
                           (+A = Counter-clockwise)
        """
        self.__target_angle = math.radians(angle)

    def get_target_angle(self) -> float:
        """
        Returns:
            float: The bot's target angle (degrees) relative to init.
                   (+A = Counter-clockwise)
        """
        return math.degrees(self.__target_angle)

    def get_angle(self) -> float:
        """
        Returns:
            float: The bot's estimated angle (degrees) relative to init.
                   (+A = Counter-clockwise)
        """
        return math.degrees(self.__angle)

    def move_towards(self, x: float, y: float) -> None:
        """
        Sets the bot's target position within arbitrary bot-space.

        Args:
            x (float): X position component. (+X = Right relative to init)
            y (float): Y position component. (+Y = Forward relative to init)
        """
        self.__target_pos.x = x
        self.__target_pos.y = y

    def move_relative(self, dx: float, dy: float, dangle: float) -> None:
        """
        Sets the bot's target position relative to its current position within
        arbitrary bot-space (not additive to previous calls).

        Args:
            dx (float): Lateral distance component. (+X = Right)
            dy (float): Longitudinal distance component. (+Y = Forward)
            dangle (float): New angle relative to current angle (degrees).
                            (+A = Counter-clockwise)
        """
        move_rads = self.__angle + math.atan2(dy, dx)
        move_dist = math.sqrt(dx * dx + dy * dy)

        self.__lerp_past_state()

        self.__target_pos.x = self.__pos.x + move_dist * math.cos(move_rads)
        self.__target_pos.y = self.__pos.y + move_dist * math.sin(move_rads)
        self.__target_angle = self.__angle + math.radians(dangle)

    def get_target_position(self) -> Point:
        """
        Returns:
            Point: Gets the estimated position after the last update() call
                   using arbitrary distance units.
        """
        return copy(self.__target_pos)

    def get_position(self) -> Point:
        """
        Returns:
            Point: Gets the bot's current estimated position within arbitrary
                   bot space.
        """
        return copy(self.__pos)

    def home_camera_angle(self) -> None:
        """
        Sets the target angle of the camera servos to point straight forward.
        Note that these cheap servos have *a lot* of backlash.
        Angles are very rough estimates at best and accuracy is both poor and
        variable.
        """
        self.__camera_target_angle.yaw = Robot.CAMERA_HOME.yaw
        self.__camera_target_angle.pitch = Robot.CAMERA_HOME.pitch

    def set_camera_angle(self, yaw: int, pitch: int) -> None:
        """
        Sets the target angle of the camera servos.
        Note that these cheap servos have *a lot* of backlash.
        Angles are very rough estimates at best and accuracy is both poor and
        variable.

        Args:
            yaw (int): Rotation around the vertical axis. 0 faces right, 180
                       faces left. Home is 90.
            pitch (int): Rotation around the lateral axis. 0 faces down-ish,
                         90 faces up-ish. Home is 15-25.
        """
        if yaw < 0:
            yaw = 0
        if yaw > 180:
            yaw = 180
        if pitch < 0:
            pitch = 0
        if pitch > 90:
            pitch = 90
        self.__camera_target_angle.yaw = yaw
        self.__camera_target_angle.pitch = pitch

    def get_camera_angle(self) -> CameraAngle:
        """
        Returns:
            CameraAngle: Gets current camera angles.
            Note that these cheap servos have *a lot* of backlash.
            Angles are very rough estimates at best and accuracy is both poor
            and variable.
        """
        return copy(self.__camera_angle)

    def get_camera_target_angle(self) -> CameraAngle:
        """
        Returns:
            CameraAngle: Gets the target camera angles.
            Note that these cheap servos have *a lot* of backlash.
            Angles are very rough estimates at best and accuracy is both poor
            and variable.
        """
        return copy(self.__camera_target_angle)

    def get_update_time(self) -> float:
        """
        Returns:
            float: System time (seconds) of last update call. Target
            adjustments and state interpolation of the next update are made
            relative to this time.
        """
        return self.__update_time

    def update(self, period: float) -> None:
        """
        Estimates current world state based on previous movement parameters,
        then updates motor parameters based on new targets.

        Args:
            period (float): Expected time (seconds) before next update() call.
        """

        self.__lerp_past_state()
        self.__set_next_state(period)

        state = self.__motor_state
        self.__bot.Ctrl_Muto(Robot.ID_MOTOR_FL, state.fl)
        self.__bot.Ctrl_Muto(Robot.ID_MOTOR_RL, state.rl)
        self.__bot.Ctrl_Muto(Robot.ID_MOTOR_FR, state.fr)
        self.__bot.Ctrl_Muto(Robot.ID_MOTOR_RR, state.rr)

        self.__bot.Ctrl_Servo(Robot.ID_SERVO_YAW, self.__camera_target_angle.yaw)
        self.__bot.Ctrl_Servo(Robot.ID_SERVO_PITCH, self.__camera_target_angle.pitch)

        self.__camera_angle.yaw = self.__camera_target_angle.yaw
        self.__camera_angle.pitch = self.__camera_target_angle.pitch

    def __lerp_past_state(self):
        """
        Calculate current state based on previous motor parameters.
        """
        last_time = self.__update_time
        next_time = time.time()
        time_diff = next_time - last_time

        state = self.__motor_state
        rads = time_diff * state.va
        if -0.0001 < rads < 0.0001:
            self.__pos.x += time_diff * state.vx
            self.__pos.y += time_diff * state.vy
            self.__angle += rads
            return

        move_angle = math.atan2(state.vy, state.vx)
        dist = time_diff * math.sqrt(state.vx * state.vx + state.vy * state.vy)

        if rads == 0 or dist == 0:
            radius = 0
            normal_scale = 0
        else:
            radius = abs(dist / rads)
            normal_scale = time_diff * radius / dist

        if rads > 0:  # left turn
            dx = normal_scale * -state.vy + radius * math.sin(move_angle + rads)
            dy = normal_scale * state.vx - radius * math.cos(move_angle + rads)
        else:  # right turn
            dx = normal_scale * state.vy - radius * math.sin(move_angle + rads)
            dy = normal_scale * -state.vx - radius * math.sin(move_angle + rads)

        self.__pos.x += dx
        self.__pos.y += dy
        self.__angle += rads

        self.__update_time = next_time

    def __set_next_state(self, period: float):
        """
        Adjusts motor parameters to direct the bot towards the target position.

        Args:
            period (float): Expected time (seconds) before next update() call.
        """
        state = self.__motor_state

        dx = self.__target_pos.x - self.__pos.x
        dy = self.__target_pos.y - self.__pos.y
        da = self.__target_angle - self.__angle

        time_to_move = abs(da / Robot.ROTATION_RATE / Robot.MAX_POWER)
        if time_to_move > period:
            normal_scale = (period / time_to_move)
            da *= normal_scale

        state.va = da / period

        move_power = state.va / Robot.ROTATION_RATE
        move_power_round = round(move_power)

        # For simplicity's sake (*not* calculating an arc to move to target)
        # this will turn to target angle first, then move to target pos.

        if abs(move_power_round) > Robot.MIN_POWER:
            move_power_error = move_power_round / move_power
            da *= move_power_error

            state.vx = 0
            state.vy = 0
            state.va *= move_power_error

            state.fl = move_power_round
            state.rl = move_power_round
            state.fr = -move_power_round
            state.rr = -move_power_round

            return

        # Turn power is below minimum threshold, go ahead with position update.

        dist = math.sqrt(dx * dx + dy * dy)
        time_to_move = abs(dist / Robot.MOVEMENT_RATE / Robot.MAX_POWER)
        if time_to_move > period:
            normal_scale = (period / time_to_move)
            dist *= normal_scale
            dx *= normal_scale
            dy *= normal_scale

        move_power = dist / period / Robot.MOVEMENT_RATE
        if move_power > Robot.MIN_POWER:
            state.vx = dx / period
            state.vy = dy / period
            state.va = 0

            move_angle = math.atan2(dy, dx) - self.__angle
            # Within [0.0, 1.0]
            speed = dist / period
            # Within [-1.0, 1.0]
            wheel_cos = math.cos(move_angle - Robot.WHEEL_PHASE)
            wheel_sin = math.sin(move_angle - Robot.WHEEL_PHASE)

            state.fl = round(wheel_cos * speed)
            state.rl = round(wheel_sin * speed)
            state.fr = round(wheel_sin * speed)
            state.rr = round(wheel_cos * speed)

            return

        # Move power is below minimum threshold, just idle.

        state.vx = 0
        state.vy = 0
        state.va = 0

        state.fl = 0
        state.rl = 0
        state.fr = 0
        state.rr = 0
