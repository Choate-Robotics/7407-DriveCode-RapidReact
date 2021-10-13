import ctre
import wpilib
import commands2 as commands

from robot_lib.command import requires, Command
from robot_systems import Robot
from utils.math import clamp
from utils.network import Network
import subsystem


@requires(Robot.turret)
class TurretAim(Command):
    def __init__(self) -> None:
        super().__init__()
        self.target_heading = 0
        self.target_position = 0
        self.start_position = 0

    def initialize(self) -> None:
        Network.limelight_table.putNumber("ledMode", 3)
        Network.limelight_table.putNumber("camMode", 0)

    def execute(self) -> None:
        self.start_position = Robot.turret.motor.getSelectedSensorPosition()
        self.target_heading = Network.limelight_table.getNumber("tx", 0.0)
        self.target_position = self.start_position + self.target_heading * (4096.0 / 360.0) * (120.0 / 16.0)

        self.target_position = clamp(
            self.target_position,
            -80.0 * (4096.0 / 360.0) * (120.0 / 16.0),
            80.0 * (4096.0 / 360.0) * (120.0 / 16.0)
        )

        Robot.turret.motor.set(ctre.ControlMode.MotionMagic, self.target_position)

    def end(self, interrupted: bool) -> None:
        Robot.turret.motor.set(ctre.ControlMode.MotionMagic, 0)
        Network.limelight_table.putNumber("ledMode", 1)
        Network.limelight_table.putNumber("camMode", 1)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
