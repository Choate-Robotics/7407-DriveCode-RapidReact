import wpilib
import commands2 as commands

import subsystem


class ShifterSolenoidControlCommand(commands.CommandBase):
    SOLENOID_STATE: bool

    def __init__(self, shifter: subsystem.Shifter) -> None:
        super().__init__()
        self.addRequirements(shifter)
        self._shifter = shifter

    def initialize(self) -> None:
        self._shifter.solenoid.set(self.SOLENOID_STATE)

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
