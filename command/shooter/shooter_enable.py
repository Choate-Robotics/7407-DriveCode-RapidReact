import ctre
import wpilib
import commands2 as commands

import subsystem
from robot_lib.command import requires, Command
from robot_systems import Robot


@requires(Robot.shooter)
class ShooterEnable(Command):
    def initialize(self) -> None:
        Robot.shooter.hood.set(wpilib.DoubleSolenoid.Value.kForward
                               if Robot.shooter.shooter_settings[1]
                               else wpilib.DoubleSolenoid.Value.kReverse)

    def execute(self) -> None:
        Robot.shooter.shooter1.set(ctre.ControlMode.Velocity, Robot.shooter.shooter_settings[0])

    def end(self, interrupted: bool) -> None:
        Robot.shooter.shooter1.set(ctre.ControlMode.Velocity, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
