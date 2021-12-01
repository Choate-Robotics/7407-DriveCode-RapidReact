from dataclasses import dataclass

from lib.motors.rev_motors import SparkMax
from lib.subsystem_templates.drivetrain.differential_drivetrain import DifferentialDrivetrain
from lib.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain, SwerveNode
from oi.keymap import Keymap


# @dataclass
# class SparkMaxSwerveNode(SwerveNode):
#     m_move: SparkMax
#     m_rotate: SparkMax
#
#     def init(self):
#         self.m_move.init()
#         self.m_rotate.init()
#
#     def set_angle_raw(self, pos: float):
#         pass
#
#     def set_velocity_raw(self, vel_tw_per_second: float):
#         pass


class Drivetrain(DifferentialDrivetrain):
    m_left = SparkMax(10)
    m_right = SparkMax(11)
    axis_x = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_y = Keymap.Drivetrain.DRIVE_Y_AXIS
