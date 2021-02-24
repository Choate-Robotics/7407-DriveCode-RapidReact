import wpilib
import commands2 as commands

import subsystem


class IntakeUp(commands.CommandBase):
    def __init__(self, intake: subsystem.Intake) -> None:
        super().__init__()
        self.addRequirements(intake)
        self._intake = intake

    def initialize(self) -> None:
        self._intake.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return False


class IntakeDown(commands.CommandBase):
    def __init__(self, intake: subsystem.Intake) -> None:
        super().__init__()
        self.addRequirements(intake)
        self._intake = intake

    def initialize(self) -> None:
        self._intake.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return False
