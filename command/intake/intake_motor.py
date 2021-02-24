import commands2 as commands

import subsystem


class IntakeRun(commands.CommandBase):
    def __init__(self, intake: subsystem.Intake) -> None:
        super().__init__()
        self.addRequirements(intake)
        self._intake = intake

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self._intake.motor.set(-0.95)

    def end(self, interrupted: bool) -> None:
        self._intake.motor.set(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class IntakeRunReverse(commands.CommandBase):
    def __init__(self, intake: subsystem.Intake) -> None:
        super().__init__()
        self.addRequirements(intake)
        self._intake = intake

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self._intake.motor.set(1.0)

    def end(self, interrupted: bool) -> None:
        self._intake.motor.set(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
