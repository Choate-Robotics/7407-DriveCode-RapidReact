from typing import Tuple
import commands2 as commands

import subsystem
from robot_lib.command import Command, requires
from robot_systems import robot


@requires(robot.shooter)
class ShooterSettingsCommand(Command):
    SHOOTER_SETTINGS: Tuple[float, bool]

    def initialize(self) -> None: robot.shooter.shooter_settings = self.SHOOTER_SETTINGS
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
