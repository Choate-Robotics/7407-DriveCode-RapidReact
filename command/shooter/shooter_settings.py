from typing import Tuple
import commands2 as commands

import subsystem


class ShooterSettingsCommand(commands.CommandBase):
    def __init__(self, shooter: subsystem.Shooter) -> None:
        super().__init__()
        self.addRequirements(shooter)
        self._shooter = shooter

    def get_shooter_settings(self) -> Tuple[float, bool]: ...

    def initialize(self) -> None: self._shooter.shooter_settings = self.get_shooter_settings()
    def execute(self) -> None: ...
    def end(self, interrupted: bool) -> None: ...
    def isFinished(self) -> bool: return True
    def runsWhenDisabled(self) -> bool: return True


class ShooterLowRetracted(ShooterSettingsCommand):
    def get_shooter_settings(self) -> Tuple[float, bool]:
        return 1000, False


class ShooterHighRetracted(ShooterSettingsCommand):
    def get_shooter_settings(self) -> Tuple[float, bool]:
        return 15000, False


class ShooterLowExtended(ShooterSettingsCommand):
    def get_shooter_settings(self) -> Tuple[float, bool]:
        return 15500, False


class ShooterHighExtended(ShooterSettingsCommand):
    def get_shooter_settings(self) -> Tuple[float, bool]:
        return 12000, True
