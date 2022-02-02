import math
from dataclasses import dataclass

import ctre
import rev
from robotpy_toolkit_7407.motors import SparkMaxConfig, SparkMax
from robotpy_toolkit_7407.sensors.gyro import GyroADIS16448
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveOdometry, SwerveDrivetrain
from robotpy_toolkit_7407.utils import logger

from oi.keymap import Keymap

TURN_CONFIG = SparkMaxConfig(0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake)
MOVE_CONFIG = SparkMaxConfig(0.00005, 0, 0.0004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake)


@dataclass
class SparkMaxSwerveNode(SwerveNode):
    m_move: SparkMax
    m_rotate: SparkMax
    encoder: ctre.CANCoder
    flipped: bool

    def init(self):
        super().init()
        self.m_move.init()
        self.m_rotate.init()

    def set_angle_raw(self, pos: float):
        self.m_rotate.set_target_position(-pos * 12.8 / (2 * math.pi))

    def set_velocity_raw(self, vel_tw_per_second: float):
        v = vel_tw_per_second / 8
        if self.motor_reversed:
            v *= -1
        if self.flipped:
            v *= -1
        self.m_move.set_raw_output(v)

    def get_current_angle_raw(self) -> float:
        return -self.m_rotate.get_sensor_position() / (12.8 / (2 * math.pi))

    def get_current_velocity(self) -> float:
        return self.m_move.get_sensor_velocity()


class GyroOdometry(SwerveOdometry):
    def __init__(self):
        self._gyro = GyroADIS16448()

    def init(self):
        self._gyro.reset()

    def get_robot_angle_degrees(self) -> float:
        return -self._gyro.angle

    def reset_angle(self):
        self._gyro.reset()


class Drivetrain(SwerveDrivetrain):
    n_00 = SparkMaxSwerveNode(
        SparkMax(7, config=MOVE_CONFIG),
        SparkMax(8, config=TURN_CONFIG),
        ctre.CANCoder(12), True
    )
    n_01 = SparkMaxSwerveNode(
        SparkMax(1, config=MOVE_CONFIG),
        SparkMax(2, config=TURN_CONFIG),
        ctre.CANCoder(9), False
    )
    n_10 = SparkMaxSwerveNode(
        SparkMax(5, config=MOVE_CONFIG),
        SparkMax(6, config=TURN_CONFIG),
        ctre.CANCoder(11), False
    )
    n_11 = SparkMaxSwerveNode(
        SparkMax(3, config=MOVE_CONFIG),
        SparkMax(4, config=TURN_CONFIG),
        ctre.CANCoder(10), True
    )
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    odometry = GyroOdometry()
