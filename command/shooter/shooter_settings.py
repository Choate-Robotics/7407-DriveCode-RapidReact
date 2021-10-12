from typing import Tuple
import commands2 as commands

import subsystem


class ShooterSettingsCommand(commands.CommandBase):
    SHOOTER_SETTINGS: Tuple[float, bool]

    def __init__(self, shooter: subsystem.Shooter) -> None:
        super().__init__()
        self.addRequirements(shooter)
        self._shooter = shooter

    def initialize(self) -> None: self._shooter.shooter_settings = self.SHOOTER_SETTINGS
    def execute(self) -> None: ...
    def end(self, interrupted: bool) -> None: ...
    def isFinished(self) -> bool: return True
    def runsWhenDisabled(self) -> bool: return True


class ShooterLowRetracted(ShooterSettingsCommand):
    SHOOTER_SETTINGS = (1000, False)


class ShooterHighRetracted(ShooterSettingsCommand):
    SHOOTER_SETTINGS = (15000, False)


class ShooterLowExtended(ShooterSettingsCommand):
    SHOOTER_SETTINGS = (15500, False)


class ShooterHighExtended(ShooterSettingsCommand):
    SHOOTER_SETTINGS = (12000, True)
