import math
from dataclasses import dataclass

from ctre import CANCoder, Pigeon2
from robotpy_toolkit_7407.motors import TalonFX, TalonConfig, ctre_motors
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveDrivetrain, SwerveGyro
from robotpy_toolkit_7407.utils.units import rad, deg, s, radians, \
    meters_per_second, radians_per_second, meters
from wpimath.geometry import Pose2d

import constants
from oi.keymap import Keymap
from utils.can_optimizations import optimize_normal_talon

TURN_IZone = 1000
TURN_kI = 0.01
TURN_I_MaxAccum = 10000
TURN_FF = 20818
TURN_kF = 1023 / TURN_FF
TURN_Cruise_Vel = TURN_FF / 2
TURN_kP = 1.1
TURN_kD = 5
TURN_CONFIG = TalonConfig(
    TURN_kP, TURN_kI, TURN_kD, TURN_kF,
    motion_cruise_velocity=17000*ctre_motors.k_sensor_vel_to_rad_per_sec,
    motion_acceleration=170000*ctre_motors.k_sensor_accel_to_rad_per_sec_sq,
    neutral_brake=True,
    integral_zone=TURN_IZone, max_integral_accumulator=TURN_I_MaxAccum
)

MOVE_IZone = 1000
MOVE_I_MaxAccum = 10000
MOVE_kP = 0.018
MOVE_kI = 0.0005
MOVE_kD = 0.5
MOVE_FF = 22365
MOVE_kF = 1023 / MOVE_FF
MOVE_CONFIG = TalonConfig(
    MOVE_kP, MOVE_kI, MOVE_kD, MOVE_kF, neutral_brake=True,
    integral_zone=MOVE_IZone, max_integral_accumulator=MOVE_I_MaxAccum
)


@dataclass
class TalonFXSwerveNode(SwerveNode):
    m_move: TalonFX
    m_turn: TalonFX
    encoder: CANCoder
    encoder_zeroed_absolute_pos_radians: radians = 0

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()
        optimize_normal_talon(self.m_move)
        optimize_normal_talon(self.m_turn)
        self.zero()

    # make the turn motor set their sensor 0 to current horizontal thingy
    def zero(self):
        current_absolute_pos_radians = math.radians(self.encoder.getAbsolutePosition())
        new_sensor_pos_radians = current_absolute_pos_radians - self.encoder_zeroed_absolute_pos_radians
        self.m_turn.set_sensor_position(new_sensor_pos_radians * constants.drivetrain_turn_gear_ratio)

    # reposition the wheels
    def set_motor_angle(self, pos: radians):
        self.m_turn.set_target_position(pos * constants.drivetrain_turn_gear_ratio)

    def get_current_motor_angle(self) -> radians:
        return self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio

    # rotate the wheel so the robot moves
    def set_motor_velocity(self, vel: meters_per_second):
        self.m_move.set_target_velocity(vel * constants.drivetrain_move_gear_ratio)

    def get_motor_velocity(self) -> radians_per_second:
        return self.m_move.get_sensor_velocity() / constants.drivetrain_move_gear_ratio


@dataclass
class ManuallyZeroedSwerveNode(SwerveNode):
    m_move: TalonFX
    m_turn: TalonFX
    # encoder: CANCoder
    # encoder_zeroed_absolute_pos_radians: Unum = 0 * rad

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()
        optimize_normal_talon(self.m_move)
        optimize_normal_talon(self.m_turn)
        self.zero()

    # make the turn motor set their sensor 0 to current horizontal thingy
    def zero(self):
        # current_absolute_pos_radians = self.encoder.getAbsolutePosition() * deg
        # new_sensor_pos_radians = current_absolute_pos_radians - self.encoder_zeroed_absolute_pos_radians
        self.m_turn.set_sensor_position(0 * constants.drivetrain_turn_gear_ratio)

    # reposition the wheels
    def set_motor_angle(self, pos: radians):
        self.m_turn.set_target_position(pos * constants.drivetrain_turn_gear_ratio)

    def get_current_motor_angle(self) -> radians:
        return self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio

    # rotate the wheel so the robot moves
    def set_motor_velocity(self, vel: meters_per_second):
        self.m_move.set_target_velocity(vel * constants.drivetrain_move_gear_ratio)

    def get_motor_velocity(self) -> meters_per_second:
        return self.m_move.get_sensor_velocity() / constants.drivetrain_move_gear_ratio


class PigeonIMUGyro(SwerveGyro):
    def __init__(self):
        self._gyro = Pigeon2(26)
        self._gyro.configMountPose(0, 0, 0)

    def init(self):
        self.reset_angle()

    def get_robot_heading(self) -> radians:
        return math.radians(self._gyro.getYaw())

    # reset the gyro
    def reset_angle(self):
        self._gyro.setYaw(0)


class Drivetrain(SwerveDrivetrain):
    n_00 = TalonFXSwerveNode(
        TalonFX(7, inverted=True, config=MOVE_CONFIG),
        TalonFX(15, inverted=True, config=TURN_CONFIG),
        CANCoder(12), (129.463 * deg).asNumber(rad) # 129.463
    )

    n_01 = TalonFXSwerveNode(
        TalonFX(1, config=MOVE_CONFIG),
        TalonFX(2, inverted=True, config=TURN_CONFIG),
        CANCoder(9), (298.916 * deg).asNumber(rad) # 298.916
    )
    n_10 = TalonFXSwerveNode(
        TalonFX(27, inverted=True, config=MOVE_CONFIG),
        TalonFX(6, inverted=True, config=TURN_CONFIG),
        CANCoder(11), (138.516 * deg).asNumber(rad) # 138.516
    )
    n_11 = TalonFXSwerveNode(
        TalonFX(3, config=MOVE_CONFIG),
        TalonFX(4, inverted=True, config=TURN_CONFIG),
        CANCoder(10), (313.506 * deg).asNumber(rad) # 313.506
    )
    gyro = PigeonIMUGyro()
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    axis_y2 = Keymap.Drivetrain.DRIVE_Y2_AXIS
    track_width: meters = constants.track_width
    max_vel: meters_per_second = constants.drivetrain_max_vel
    max_angular_vel: radians_per_second = constants.drivetrain_max_angular_vel
    deadzone_velocity: meters_per_second = 0.01
    deadzone_angular_velocity: radians_per_second = (5 * deg/s).asNumber(rad/s)
    start_pose: Pose2d = Pose2d(0, 0, 0)
