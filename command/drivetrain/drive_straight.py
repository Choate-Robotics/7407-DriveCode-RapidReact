from typing import Tuple
import commands2 as commands
import ctre

import subsystem
from oi import OI
from oi.joysticks import Joysticks
from oi.keymap import Keymap
from robot_systems import Robot
from robot_lib.command import Command, requires
from utils.math import sensor_units_to_inches, inches_to_sensor_units, clamp
import utils.logger as logger

import constants


@requires(Robot.drivetrain)
class DriveStraight(Command):
    def __init__(self, motor_vel: float):
        super().__init__()
        self._motor_vel = motor_vel

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        Robot.drivetrain.set_motor_velocity(self._motor_vel, -self._motor_vel)

    def end(self, interrupted: bool) -> None:
        Robot.drivetrain.set_motor_velocity(0, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
