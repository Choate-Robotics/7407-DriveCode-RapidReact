from robotpy_toolkit_7407.command import SubsystemCommand

from robot_systems import Robot
from subsystem import Shooter


class ShooterEnable(SubsystemCommand[Shooter]):
    def initialize(self) -> None:
        self.subsystem.target(Robot.limelight.calculate_distance())

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()
