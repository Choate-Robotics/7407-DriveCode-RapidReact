import ctre
import commands2 as commands

import subsystem
from robot_lib.command import Command, requires
from robot_systems import Robot


@requires(Robot.hopper)
class HopperControlCommand(Command):
    MOTOR_SPEED: float

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        Robot.hopper.motor.set(ctre.ControlMode.PercentOutput, self.MOTOR_SPEED)

    def end(self, interrupted: bool) -> None:
        Robot.hopper.motor.set(ctre.ControlMode.PercentOutput, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class HopperRun(HopperControlCommand):
    MOTOR_SPEED = -0.5


class HopperRunReverse(HopperControlCommand):
    MOTOR_SPEED = 0.5
