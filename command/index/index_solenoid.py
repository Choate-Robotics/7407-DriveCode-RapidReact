import wpilib
import commands2 as commands

import subsystem
from robot_lib.command import Command, requires
from robot_systems import Robot


@requires(Robot.index)
class IndexSolenoidControlCommand(Command):
    SOLENOID_STATE: wpilib.DoubleSolenoid.Value

    def initialize(self) -> None:
        Robot.index.solenoid.set(self.SOLENOID_STATE)

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return False


class IndexUp(IndexSolenoidControlCommand):
    SOLENOID_STATE = wpilib.DoubleSolenoid.Value.kForward


class IndexDown(IndexSolenoidControlCommand):
    SOLENOID_STATE = wpilib.DoubleSolenoid.Value.kReverse
