import wpilib
import commands2 as commands

import subsystem
from robot_lib.command import requires, Command
from robot_systems import Robot


@requires(Robot.shifter)
class ShifterSolenoidControlCommand(Command):
    SOLENOID_STATE: bool

    def initialize(self) -> None:
        # Robot.shifter.solenoid.set(self.SOLENOID_STATE)
        pass

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return False


class ShifterHighGear(ShifterSolenoidControlCommand):
    SOLENOID_STATE = True


class ShifterLowGear(ShifterSolenoidControlCommand):
    SOLENOID_STATE = False
