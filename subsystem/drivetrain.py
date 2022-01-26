import math
from dataclasses import dataclass

from ctre import ControlMode, CANCoder
from robotpy_toolkit_7407.motors import TalonFX, TalonConfig
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveOdometry, SwerveDrivetrain
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import bounded_angle_diff

from oi.keymap import Keymap


TURN_FF = 20818
TURN_kF = 1023 / TURN_FF
TURN_Cruise_Vel = TURN_FF / 2
TURN_kP = 0.0957142857
TURN_kD = 1.5
TURN_CONFIG = TalonConfig(TURN_kP, 0, TURN_kD, TURN_kF, neutral_brake=True)

MOVE_FF = 21273
MOVE_kF = 1023 / MOVE_FF
MOVE_CONFIG = TalonConfig(0, 0, 0, MOVE_kF, neutral_brake=True)


@dataclass
class TalonFXSwerveNode(SwerveNode):
    m_move: TalonFX
    m_turn: TalonFX
    encoder: CANCoder
    encoder_zeroed_absolute_pos_radians: float = 0
    _setpoint: float = 0
    _zero_offset: float = 0

    __gear_ratio = (2048 / (2 * math.pi)) * 6.55

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()
        current_absolute_pos_radians = self.encoder.getAbsolutePosition() * 0.017453292519943295  # degrees to radians
        current_sensor_pos_radians = self.m_turn.get_sensor_position() / TalonFXSwerveNode.__gear_ratio
        self._zero_offset = bounded_angle_diff(
            current_absolute_pos_radians - current_sensor_pos_radians,
            self.encoder_zeroed_absolute_pos_radians
        )

    def set_angle_raw(self, pos: float):
        pos += self._zero_offset
        self._setpoint = pos
        self.m_turn.set_target_position(pos * TalonFXSwerveNode.__gear_ratio)

    def set_velocity_raw(self, vel_tw_per_second: float):
        if self.motor_reversed:
            self.m_move.set_raw_output(-vel_tw_per_second / 8)
        else:
            self.m_move.set_raw_output(vel_tw_per_second / 8)

    def get_current_angle_raw(self) -> float:
        return self._setpoint - self._zero_offset
        # return (self.m_turn.get_sensor_position() / TalonFXSwerveNode.__gear_ratio) - self.encoder_zero_offset_radians

    def get_current_velocity(self) -> float:
        return self.m_move.get_sensor_velocity()


# TODO Add Pigeon IMU
class GyroOdometry(SwerveOdometry):
    def __init__(self):
        # self._gyro = GyroADIS16448()
        pass

    def init(self):
        # self._gyro.reset()
        pass

    def get_robot_angle_degrees(self) -> float:
        # return -self._gyro.angle
        return 0

    def reset_angle(self):
        # self._gyro.reset()
        pass


class Drivetrain(SwerveDrivetrain):
    n_00 = TalonFXSwerveNode(
        TalonFX(7, config=MOVE_CONFIG),
        TalonFX(8, inverted=True, config=TURN_CONFIG),
        CANCoder(12)
    )
    n_01 = TalonFXSwerveNode(
        TalonFX(1, inverted=True, config=MOVE_CONFIG),
        TalonFX(2, inverted=True, config=TURN_CONFIG),
        CANCoder(9)
    )
    n_10 = TalonFXSwerveNode(
        TalonFX(5, config=MOVE_CONFIG),
        TalonFX(6, inverted=True, config=TURN_CONFIG),
        CANCoder(11)
    )
    n_11 = TalonFXSwerveNode(
        TalonFX(3, inverted=True, config=MOVE_CONFIG),
        TalonFX(4, inverted=True, config=TURN_CONFIG),
        CANCoder(10)
    )
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    odometry = GyroOdometry()
