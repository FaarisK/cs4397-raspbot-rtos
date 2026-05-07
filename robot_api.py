import cv2
import math
import smbus
import numpy
import time
from copy import copy


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


class TrackState:
    def __init__(self, left: bool, midleft: bool, midright: bool, right: bool):
        self.left = left
        self.midleft = midleft
        self.midright = midright
        self.right = right


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

    I2C_ADDRESS = 0x2b
    I2C_BUS = 0x01

    # I2C Register Values:
    # 0x01 -W [n, d, s]     Write motor n (1..4) to speed s (0..255) in direction d (0,1)
    # 0x02 -W [n, a]        Write servo n (1..2) to angle a (1: 0..180, 2: 0..90)
    # 0x03 -W [e]           Write lightbar        enum value e (0..7)
    # 0x04 -W [n, e]        Write light n (1..10) enum value e (0..7)
    # 0x05 -W [x]           Write IR reader  enabled state x (F/T)
    # 0x06 -W [x]           Write buzzer     enabled state x (F/T)
    # 0x07 -W [x]           Write ultrasonic enabled state x (F/T)
    # 0x08 -W [R, G, B]     Write lightbar RGB value
    # 0x09 -W [n, R, G, B]  Write light n (1..10) RGB value
    # 0x0a R- [lLrR]        Read IR track bitfield (left to right: L l r R)
    # ---- --
    # 0x0c R- [i]           Read IR reader value i (0..255) [See IR Remote Reference]
    # 0x0d R- [k]           Read rear KEY1 value (F/T)
    # ---- --
    # 0x1a R- [L]           Read ultrasonic distance low-bits (mm)
    # 0x1b R- [H]           Read ultrasonic distance high-bits (mm)
    # ---- --
    REG_MOTOR = 0x01
    REG_SERVO = 0x02
    REG_LED_ENUM_ALL = 0x03
    REG_LED_ENUM_ONE = 0x04
    REG_IR_SWITCH = 0x05
    REG_BUZZER_SWITCH = 0x06
    REG_ULTRASONIC_SWITCH = 0x07
    REG_LED_RGB_ALL = 0x08
    REG_LED_RGB_ONE = 0x09
    REG_IR_TRACK = 0x0a
    REG_IR_READER = 0x0c
    REG_KEY1 = 0x0d
    REG_ULTRASONIC_LOW = 0x1a
    REG_ULTRASONIC_HIGH = 0x1b

    # Motor / Servo IDs:
    ID_MOTOR_FL = 0
    ID_MOTOR_RL = 1
    ID_MOTOR_FR = 2
    ID_MOTOR_RR = 3
    ID_SERVO_YAW = 1
    ID_SERVO_PITCH = 2

    # IR Remote Reference:
    # +--------+  +--------------+
    # |00 01 02|  |PWR   ^^  LIGT|
    # |04 05 06|  | <<  SPKR  >> |
    # |08 09 0a|  | <U   vv   U> |
    # |0c 0d 0e|  | ++   00   -- |
    # |10 11 12|  | 11   22   33 |
    # |14 15 16|  | 44   55   66 |
    # |18 19 1a|  | 77   88   99 |
    # +--------+  +--------------+
    RMT_POWER = 0x00
    RMT_UP = 0x01
    RMT_LIGHT = 0x02
    RMT_LEFT = 0x04
    RMT_SPEAKER = 0x05
    RMT_RIGHT = 0x06
    RMT_TURN_LEFT = 0x08
    RMT_DOWN = 0x09
    RMT_TURN_RIGHT = 0x0a
    RMT_PLUS = 0x0c
    RMT_0 = 0x0d
    RMT_MINUS = 0x0e
    RMT_1 = 0x10
    RMT_2 = 0x11
    RMT_3 = 0x12
    RMT_4 = 0x14
    RMT_5 = 0x15
    RMT_6 = 0x16
    RMT_7 = 0x18
    RMT_8 = 0x19
    RMT_9 = 0x1a

    # Front Track Bitmasks:
    TRACK_LEFTLEFT = 0b0100
    TRACK_MIDLEFT = 0b1000
    TRACK_MIDRIGHT = 0b0010
    TRACK_RIGHTRIGHT = 0b0001

    def set_rate_calibration(self, movement: float = 1.0, rotation: float = 2.25) -> None:
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
        self._i2c_device = smbus.SMBus(Robot.I2C_BUS)
        self._i2c_buf_servo = [
            [Robot.ID_SERVO_YAW, 0],
            [Robot.ID_SERVO_PITCH, 0]
        ]
        self._i2c_buf_motor = [
            [Robot.ID_MOTOR_FL, 0, 0],
            [Robot.ID_MOTOR_RL, 0, 0],
            [Robot.ID_MOTOR_FR, 0, 0],
            [Robot.ID_MOTOR_RR, 0, 0]
        ]
        self._i2c_buf2 = [0, 0]
        self._i2c_buf3 = [0, 0, 0]
        self._i2c_buf4 = [0, 0, 0, 0]
        self.reset()

    def __del__(self):
        self.reset()
        self._i2c_device.close()

    def reset(self) -> None:
        """
        Zeros the motors, homes the camera servos, disables lights and
        switches, and zeros the estimated and target states.
        """
        self.write_motors(0, 0, 0, 0)
        self.write_servos(Robot.CAMERA_HOME.yaw, Robot.CAMERA_HOME.pitch)
        self.write_led_rgb_all(0, 0, 0)
        self.write_led_enum_all(7)
        self.toggle_buzzer(False)
        self.toggle_ir_reader(False)
        self.toggle_ultrasonic(False)

        self._motor_state = MotorState()
        self._pos = Point(0.0, 0.0)
        self._target_pos = Point(0.0, 0.0)
        self._angle = 0.0
        self._target_angle = 0.0
        self._camera_angle = CameraAngle(Robot.CAMERA_HOME.yaw, Robot.CAMERA_HOME.pitch)
        self._camera_target_angle = CameraAngle(Robot.CAMERA_HOME.yaw, Robot.CAMERA_HOME.pitch)
        self._update_time = time.monotonic()

    def get_motor_state(self):
        """
        Returns:
            MotorState: A copy of the motor state after the last update() call.
        """
        return copy(self._motor_state)

    def turn_towards(self, angle: float) -> None:
        """
        Sets the bot's target angle.

        Args:
            angle (float): New target angle (degrees) relative to init.
                           (+A = Counter-clockwise)
        """
        self._target_angle = math.radians(angle)

    def get_target_angle(self) -> float:
        """
        Returns:
            float: The bot's target angle (degrees) relative to init.
                   (+A = Counter-clockwise)
        """
        return math.degrees(self._target_angle)

    def get_angle(self) -> float:
        """
        Returns:
            float: The bot's estimated angle (degrees) relative to init.
                   (+A = Counter-clockwise)
        """
        return math.degrees(self._angle)

    def move_towards(self, x: float, y: float) -> None:
        """
        Sets the bot's target position within arbitrary bot-space.

        Args:
            x (float): X position component. (+X = Right relative to init)
            y (float): Y position component. (+Y = Forward relative to init)
        """
        self._target_pos.x = x
        self._target_pos.y = y

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
        move_rads = self._angle + math.atan2(dy, dx)
        move_dist = math.sqrt(dx * dx + dy * dy)

        self.__lerp_past_state()

        self._target_pos.x = self._pos.x + move_dist * math.cos(move_rads)
        self._target_pos.y = self._pos.y + move_dist * math.sin(move_rads)
        self._target_angle = self._angle + math.radians(dangle)

    def get_target_position(self) -> Point:
        """
        Returns:
            Point: Gets the estimated position after the last update() call
                   using arbitrary distance units.
        """
        return copy(self._target_pos)

    def get_position(self) -> Point:
        """
        Returns:
            Point: Gets the bot's current estimated position within arbitrary
                   bot space.
        """
        return copy(self._pos)

    def home_camera_angle(self) -> None:
        """
        Sets the target angle of the camera servos to point straight forward.
        Note that these cheap servos have *a lot* of backlash.
        Angles are very rough estimates at best and accuracy is both poor and
        variable.
        """
        self._camera_target_angle.yaw = Robot.CAMERA_HOME.yaw
        self._camera_target_angle.pitch = Robot.CAMERA_HOME.pitch

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
        self._camera_target_angle.yaw = yaw
        self._camera_target_angle.pitch = pitch

    def get_camera_angle(self) -> CameraAngle:
        """
        Returns:
            CameraAngle: Gets current camera angles.
            Note that these cheap servos have *a lot* of backlash.
            Angles are very rough estimates at best and accuracy is both poor
            and variable.
        """
        return copy(self._camera_angle)

    def get_camera_target_angle(self) -> CameraAngle:
        """
        Returns:
            CameraAngle: Gets the target camera angles.
            Note that these cheap servos have *a lot* of backlash.
            Angles are very rough estimates at best and accuracy is both poor
            and variable.
        """
        return copy(self._camera_target_angle)

    def get_update_time(self) -> float:
        """
        Returns:
            float: System time (seconds) of last update call. Target
            adjustments and state interpolation of the next update are made
            relative to this time.
        """
        return self._update_time

    def update(self, period: float) -> None:
        """
        Estimates current world state based on previous movement parameters,
        then updates motor parameters based on new targets.

        Args:
            period (float): Expected time (seconds) before next update() call.
        """

        self.__lerp_past_state()
        self.__set_next_state(period)

        state = self._motor_state
        self.write_motors(state.fl, state.rl, state.fr, state.rr)
        self.write_servos(self._camera_target_angle.yaw, self._camera_target_angle.pitch)

    def __lerp_past_state(self) -> None:
        """
        Calculate current state based on previous motor parameters.
        """
        last_time = self._update_time
        next_time = time.monotonic()
        time_diff = next_time - last_time

        state = self._motor_state
        rads = time_diff * state.va
        if -0.0001 < rads < 0.0001:
            self._pos.x += time_diff * state.vx
            self._pos.y += time_diff * state.vy
            self._angle += rads
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

        self._pos.x += dx
        self._pos.y += dy
        self._angle += rads

        self._update_time = next_time

    def __set_next_state(self, period: float) -> None:
        """
        Adjusts motor parameters to direct the bot towards the target position.

        Args:
            period (float): Expected time (seconds) before next update() call.
        """
        state = self._motor_state

        dx = self._target_pos.x - self._pos.x
        dy = self._target_pos.y - self._pos.y
        da = self._target_angle - self._angle

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

            move_angle = math.atan2(dy, dx) - self._angle
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

    def write_motors(self, fl: int, rl: int, fr: int, rr: int) -> None:
        """
        Updates all 4 motors with low latency between each to minimize error.

        Args:
            fl (int): Front left wheel power [-255..255]
            rl (int): Rear left wheel power [-255..255]
            fr (int): Front right wheel power [-255..255]
            rr (int): Rear right wheel power [-255..255]
        """
        dev = self._i2c_device
        buf = self._i2c_buf_motor

        if fl > Robot.MAX_POWER:
            fl = Robot.MAX_POWER
        elif fl < -Robot.MAX_POWER:
            fl = -Robot.MAX_POWER
        buf[0][1] = fl < 0
        buf[0][2] = abs(fl)

        if rl > Robot.MAX_POWER:
            rl = Robot.MAX_POWER
        elif rl < -Robot.MAX_POWER:
            rl = -Robot.MAX_POWER
        buf[1][1] = rl < 0
        buf[1][2] = abs(rl)

        if fr > Robot.MAX_POWER:
            fr = Robot.MAX_POWER
        elif fr < -Robot.MAX_POWER:
            fr = -Robot.MAX_POWER
        buf[2][1] = fr < 0
        buf[2][2] = abs(fr)

        if rr > Robot.MAX_POWER:
            rr = Robot.MAX_POWER
        elif rr < -Robot.MAX_POWER:
            rr = -Robot.MAX_POWER
        buf[3][1] = rr < 0
        buf[3][2] = abs(rr)

        try:
            dev.write_i2c_block_data(Robot.I2C_ADDRESS, Robot.REG_MOTOR, buf[0])
            dev.write_i2c_block_data(Robot.I2C_ADDRESS, Robot.REG_MOTOR, buf[1])
            dev.write_i2c_block_data(Robot.I2C_ADDRESS, Robot.REG_MOTOR, buf[2])
            dev.write_i2c_block_data(Robot.I2C_ADDRESS, Robot.REG_MOTOR, buf[3])
        except Exception as e:
            print("[ERROR] I2C error while writing motor data", e)

    def write_servos(self, yaw: int, pitch: int) -> None:
        """
        Updates both camera servos.

        Args:
            yaw (int): Rotation around the vertical axis [0..180]
            pitch (int): Rotation around the horiozntal axis [0..90]
        """
        dev = self._i2c_device
        buf = self._i2c_buf_servo

        if yaw < Robot.CAMERA_YAW_MIN:
            yaw = Robot.CAMERA_YAW_MIN
        elif yaw > Robot.CAMERA_YAW_MAX:
            yaw = Robot.CAMERA_YAW_MAX
        buf[0][1] = yaw

        if pitch < Robot.CAMERA_PITCH_MIN:
            pitch = Robot.CAMERA_PITCH_MIN
        if pitch > Robot.CAMERA_PITCH_MAX:
            pitch = Robot.CAMERA_PITCH_MAX
        buf[1][1] = pitch

        try:
            dev.write_i2c_block_data(Robot.I2C_ADDRESS, Robot.REG_SERVO, buf[0])
            dev.write_i2c_block_data(Robot.I2C_ADDRESS, Robot.REG_SERVO, buf[1])
        except Exception as e:
            print("[ERROR] I2C error while writing servo data", e)

    def write_led_enum_all(self, value: int) -> None:
        """
        Updates all LED colors using a predefined enum value.

        Args:
            value (int): Color preset [0..7]
        """
        dev = self._i2c_device
        buf = self._i2c_buf2

        buf[0] = 0 <= value <= 6
        buf[1] = value

        try:
            dev.write_i2c_block_data(Robot.I2C_ADDRESS, Robot.REG_LED_ENUM_ALL, buf)
        except Exception as e:
            print("[ERROR] I2C error while writing LED all-enum data", e)

    def write_led_enum_one(self, n: int, value: int) -> None:
        """
        Updates one LED color using a predefined enum value.

        Args:
            n (int): LED number [1..10]
            value (int): Color preset [0..7]
        """
        dev = self._i2c_device
        buf = self._i2c_buf3

        buf[0] = n
        buf[1] = 0 <= value <= 6
        buf[2] = value

        try:
            dev.write_i2c_block_data(Robot.I2C_ADDRESS, Robot.REG_LED_ENUM_ONE, buf)
        except Exception as e:
            print("[ERROR] I2C error while writing LED one-enum data", e)

    def write_led_rgb_all(self, r: int, g: int, b: int) -> None:
        """
        Updates all LED colors using a specific RGB value.

        Args:
            r (int): Red component [0..255]
            g (int): Green component [0..255]
            b (int): Blue component [0..255]
        """
        dev = self._i2c_device
        buf = self._i2c_buf3

        buf[0] = r
        buf[1] = g
        buf[2] = b

        try:
            dev.write_i2c_block_data(Robot.I2C_ADDRESS, Robot.REG_LED_RGB_ALL, buf)
        except Exception as e:
            print("[ERROR] I2C error while writing LED all-rgb data", e)

    def write_led_rgb_one(self, n: int, r: int, g: int, b: int) -> None:
        """
        Updates one LED color using a specific RGB value.

        Args:
            n (int): LED number [1..10]
            r (int): Red component [0..255]
            g (int): Green component [0..255]
            b (int): Blue component [0..255]
        """
        dev = self._i2c_device
        buf = self._i2c_buf4

        buf[0] = n
        buf[1] = r
        buf[2] = g
        buf[3] = b

        try:
            dev.write_i2c_block_data(Robot.I2C_ADDRESS, Robot.REG_LED_RGB_ONE, buf)
        except Exception as e:
            print("[ERROR] I2C error while writing LED one-rgb data", e)

    def toggle_ir_reader(self, enabled: bool) -> None:
        """
        Enables or disables the infrared remote reader.

        Args:
            enabled (bool): whether the infrared sensor is enabled.
        """
        try:
            self._i2c_device.write_byte_data(Robot.I2C_ADDRESS, Robot.REG_IR_SWITCH, enabled != False)
        except Exception as e:
            print("[ERROR] I2C error while writing IR reader state", e)

    def toggle_buzzer(self, enabled: bool) -> None:
        """
        Enables or disables the buzzer.

        Args:
            enabled (bool): whether the buzzer is enabled.
        """
        try:
            self._i2c_device.write_byte_data(Robot.I2C_ADDRESS, Robot.REG_BUZZER_SWITCH, enabled != False)
        except Exception as e:
            print("[ERROR] I2C error while writing buzzer state", e)

    def toggle_ultrasonic(self, enabled: bool) -> None:
        """
        Enables or disables the ultrasonic distance sensor.

        Args:
            enabled (bool): whether the ultrasonic distance sensor is enabled.
        """
        try:
            self._i2c_device.write_byte_data(Robot.I2C_ADDRESS, Robot.REG_ULTRASONIC_SWITCH, enabled != False)
        except Exception as e:
            print("[ERROR] I2C error while writing ultrasonic state", e)

    def read_ir_tracks(self) -> TrackState:
        """
        Returns:
            TrackState: the binary state of the four IR floor trackers.
        """
        try:
            data = self._i2c_device.read_byte_data(Robot.I2C_ADDRESS, Robot.REG_IR_TRACK)
            return TrackState(
                (data & Robot.TRACK_LEFTLEFT) != 0,
                (data & Robot.TRACK_MIDLEFT) != 0,
                (data & Robot.TRACK_MIDRIGHT) != 0,
                (data & Robot.TRACK_RIGHTRIGHT) != 0
            )
        except Exception as e:
            print("[ERROR] I2C error while reading track state", e)
            return TrackState(False, False, False, False)

    def read_ir_reader(self) -> int:
        """
        Returns:
            int: the value read from an infrared remote, or 255 if none is read.
        """
        try:
            return self._i2c_device.read_byte_data(Robot.I2C_ADDRESS, Robot.REG_IR_READER)
        except Exception as e:
            print("[ERROR] I2C error while reading IR reader state", e)
            return 0

    def read_key1(self) -> bool:
        """
        Returns:
            bool: whether the KEY1 button on the rear of the robot is pressed.
        """
        try:
            return self._i2c_device.read_byte_data(Robot.I2C_ADDRESS, Robot.REG_KEY1) != 0
        except Exception as e:
            print("[ERROR] I2C error while reading key1 state", e)
            return False

    def read_ultrasonic(self) -> int:
        """
        Returns:
            int: the distance (mm) measured by the ultrasonic sensor.
        """
        try:
            low = self._i2c_device.read_byte_data(Robot.I2C_ADDRESS, Robot.REG_ULTRASONIC_LOW)
            high = self._i2c_device.read_byte_data(Robot.I2C_ADDRESS, Robot.REG_ULTRASONIC_HIGH)
            distance = int(high) << 8 | int(low)
            return distance
        except Exception as e:
            print("[ERROR] I2C error while reading IR reader state", e)
            return 0
