import ctre
import wpilib
import commands2 as commands

import subsystem


class ShooterEnable(commands.CommandBase):
    def __init__(self, shooter: subsystem.Shooter) -> None:
        super().__init__()
        self.addRequirements(shooter)
        self._shooter = shooter

    def initialize(self) -> None:
        self._shooter.hood.set(wpilib.DoubleSolenoid.Value.kForward
                               if self._shooter.shooter_settings[1]
                               else wpilib.DoubleSolenoid.Value.kReverse)

    def execute(self) -> None:
        self._shooter.shooter1.set(ctre.ControlMode.Velocity, self._shooter.shooter_settings[0])

    def end(self, interrupted: bool) -> None:
        self._shooter.shooter1.set(ctre.ControlMode.Velocity, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
