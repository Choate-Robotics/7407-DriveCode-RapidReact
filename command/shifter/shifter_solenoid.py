import wpilib
import commands2 as commands

import subsystem


class ShifterHighGear(commands.CommandBase):
    def __init__(self, shifter: subsystem.Shifter) -> None:
        super().__init__()
        self.addRequirements(shifter)
        self._shifter = shifter

    def initialize(self) -> None:
        self._shifter.solenoid.set(True)

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return False


class ShifterLowGear(commands.CommandBase):
    def __init__(self, shifter: subsystem.Shifter) -> None:
        super().__init__()
        self.addRequirements(shifter)
        self._shifter = shifter

    def initialize(self) -> None:
        self._shifter.solenoid.set(False)

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return False
