from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Shooter


class ShooterEnable(SubsystemCommand[Shooter]):
    def initialize(self) -> None:
        self.subsystem.target(3.33*2.15)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()
