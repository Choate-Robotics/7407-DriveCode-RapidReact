import ctre
import commands2 as commands

import subsystem
from robot_lib.command import Command, requires
from robot_systems import Robot


@requires(Robot.index)
class IndexShoot(Command):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        Robot.index.top_motor.set(-0.3)
        Robot.index.bottom_motor.set(ctre.ControlMode.PercentOutput, 0.5)

    def end(self, interrupted: bool) -> None:
        Robot.index.top_motor.set(0)
        Robot.index.bottom_motor.set(ctre.ControlMode.PercentOutput, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
