from lib.command import Command, SubsystemCommand
from subsystem.intake import Intake


class IntakeRunCommand(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake, speed: float):
        super().__init__(subsystem)
        self.speed = speed

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.subsystem.set(self.speed)

    def end(self, interrupted: bool) -> None:
        self.subsystem.set(0)

    def isFinished(self) -> bool:
        return False
