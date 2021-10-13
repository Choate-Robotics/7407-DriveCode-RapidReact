import wpilib
import commands2 as commands

import subsystem
from robot_lib.command import requires, Command
from robot_systems import robot


@requires(robot.intake)
class IntakeSolenoidControlCommand(Command):
    SOLENOID_STATE: wpilib.DoubleSolenoid.Value

    def initialize(self) -> None:
        robot.intake.solenoid.set(self.SOLENOID_STATE)

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return False


class IntakeUp(IntakeSolenoidControlCommand):
    SOLENOID_STATE = wpilib.DoubleSolenoid.Value.kForward


class IntakeDown(IntakeSolenoidControlCommand):
    SOLENOID_STATE = wpilib.DoubleSolenoid.Value.kReverse
