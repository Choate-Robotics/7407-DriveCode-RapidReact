import commands2 as commands

import subsystem
from robot_lib.command import requires, Command
from robot_systems import robot


@requires(robot.intake)
class IntakeMotorControlCommand(Command):
    MOTOR_SPEED: float

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        robot.intake.motor.set(self.MOTOR_SPEED)

    def end(self, interrupted: bool) -> None:
        robot.intake.motor.set(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class IntakeRun(IntakeMotorControlCommand):
    MOTOR_SPEED = -0.95


class IntakeRunReverse(IntakeMotorControlCommand):
    MOTOR_SPEED = 1.0
