import math
from dataclasses import dataclass

from ctre import ControlMode, CANCoder, PigeonIMU
from robotpy_toolkit_7407.motors import TalonFX, TalonConfig
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit, talon_sensor_vel_unit, talon_sensor_accel_unit
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveOdometry, SwerveDrivetrain
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import bounded_angle_diff
from robotpy_toolkit_7407.utils.units import rad, rev, inch, deg
from unum import Unum

from oi.keymap import Keymap


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
    motion_cruise_velocity=17000*talon_sensor_vel_unit,
    motion_acceleration=170000*talon_sensor_accel_unit,
    neutral_brake=True,
    integral_zone=TURN_IZone, max_integral_accumulator=TURN_I_MaxAccum
)

TURN_GEAR_RATIO = 3353.33 * talon_sensor_unit/rad
MOVE_GEAR_RATIO = 1 * inch/rad  # TODO DETERMINE THIS

MOVE_IZone = 10001
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
    encoder_zeroed_absolute_pos_radians: Unum = 0 * rad

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()
        self.zero()

    def zero(self):
        current_absolute_pos_radians = self.encoder.getAbsolutePosition() * deg
        new_sensor_pos_radians = current_absolute_pos_radians - self.encoder_zeroed_absolute_pos_radians
        self.m_turn.set_sensor_position(new_sensor_pos_radians * TURN_GEAR_RATIO)

    def set_motor_angle(self, pos: Unum):
        self.m_turn.set_target_position(pos * TURN_GEAR_RATIO)

    def get_current_motor_angle(self) -> Unum:
        return self.m_turn.get_sensor_position() / TURN_GEAR_RATIO

    def set_motor_velocity(self, vel: Unum):
        self.m_move.set_target_velocity(vel * MOVE_GEAR_RATIO)

    def get_motor_velocity(self) -> Unum:
        return self.m_move.get_sensor_velocity() / MOVE_GEAR_RATIO


# TODO Add status checks
class GyroOdometry(SwerveOdometry):
    def __init__(self):
        self._gyro = PigeonIMU(13)

    def init(self):
        self.reset_angle()

    def get_robot_angle_degrees(self) -> float:
        return self._gyro.getFusedHeading()

    def reset_angle(self):
        self._gyro.setFusedHeading(0)


class Drivetrain(SwerveDrivetrain):
    n_00 = TalonFXSwerveNode(
        TalonFX(7, inverted=True, config=MOVE_CONFIG),
        TalonFX(8, inverted=True, config=TURN_CONFIG),
        CANCoder(12), 26.719 * deg
    )
    n_01 = TalonFXSwerveNode(
        TalonFX(1, config=MOVE_CONFIG),
        TalonFX(2, inverted=True, config=TURN_CONFIG),
        CANCoder(9), 316.758 * deg
    )
    n_10 = TalonFXSwerveNode(
        TalonFX(5, inverted=True, config=MOVE_CONFIG),
        TalonFX(6, inverted=True, config=TURN_CONFIG),
        CANCoder(11), 138.516 * deg
    )
    n_11 = TalonFXSwerveNode(
        TalonFX(3, config=MOVE_CONFIG),
        TalonFX(4, inverted=True, config=TURN_CONFIG),
        CANCoder(10), 313.506 * deg
    )
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    odometry = GyroOdometry()
