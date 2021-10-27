import ctre
import commands2 as commands

import subsystem
from robot_lib.command import Command, requires
from robot_systems import Robot


@requires(Robot.index)
class IndexShoot(Command):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        Robot.index.m_top.set_raw_output(-0.3)
        Robot.index.m_bottom.set_raw_output(0.5)

    def end(self, interrupted: bool) -> None:
        Robot.index.m_top.set_raw_output(0)
        Robot.index.m_bottom.set_raw_output(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
