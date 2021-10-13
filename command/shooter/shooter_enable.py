import ctre
import wpilib
import commands2 as commands

import subsystem
from robot_lib.command import requires, Command
from robot_systems import robot


@requires(robot.shooter)
class ShooterEnable(Command):
    def initialize(self) -> None:
        robot.shooter.hood.set(wpilib.DoubleSolenoid.Value.kForward
                               if robot.shooter.shooter_settings[1]
                               else wpilib.DoubleSolenoid.Value.kReverse)

    def execute(self) -> None:
        robot.shooter.shooter1.set(ctre.ControlMode.Velocity, robot.shooter.shooter_settings[0])

    def end(self, interrupted: bool) -> None:
        robot.shooter.shooter1.set(ctre.ControlMode.Velocity, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
