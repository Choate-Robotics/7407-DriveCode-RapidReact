import math
from dataclasses import dataclass

import ctre

from lib.motors.rev_motors import SparkMax, SparkMaxConfig
from lib.subsystem_templates.drivetrain.differential_drivetrain import DifferentialDrivetrain
from lib.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain, SwerveNode
from oi.keymap import Keymap
from utils import logger

TURN_CONFIG = SparkMaxConfig(0.2, 0, 0.003, 0.00015, (-0.5, 0.5))
MOVE_CONFIG = SparkMaxConfig(0.00005, 0, 0.0004, 0.00017)


@dataclass
class SparkMaxSwerveNode(SwerveNode):
    m_move: SparkMax
    m_rotate: SparkMax
    encoder: ctre.CANCoder

    def init(self):
        self.m_move.init()
        self.m_rotate.init()

    def set_angle_raw(self, pos: float):
        logger.info(f"pos {pos}")
        self.m_rotate.set_target_position(pos * 12.8 / (2 * math.pi))

    def set_velocity_raw(self, vel_tw_per_second: float):
        self.m_move.set_raw_output(vel_tw_per_second / 8)


class Drivetrain(SwerveDrivetrain):
    n_00 = SparkMaxSwerveNode(
        SparkMax(7, config=MOVE_CONFIG),
        SparkMax(8, config=TURN_CONFIG),
        ctre.CANCoder(12)
    )
    n_01 = SparkMaxSwerveNode(
        SparkMax(1, config=MOVE_CONFIG),
        SparkMax(2, config=TURN_CONFIG),
        ctre.CANCoder(9)
    )
    n_10 = SparkMaxSwerveNode(
        SparkMax(5, config=MOVE_CONFIG),
        SparkMax(6, config=TURN_CONFIG),
        ctre.CANCoder(11)
    )
    n_11 = SparkMaxSwerveNode(
        SparkMax(3, config=MOVE_CONFIG),
        SparkMax(4, config=TURN_CONFIG),
        ctre.CANCoder(10)
    )
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
