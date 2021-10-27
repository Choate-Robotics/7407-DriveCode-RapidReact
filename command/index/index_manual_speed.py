import ctre
import commands2 as commands

import subsystem
from oi import OI
from oi.joysticks import Joysticks
from oi.keymap import Keymap
from robot_lib.command import Command, requires
from robot_systems import Robot


@requires(Robot.index)
class IndexManualSpeedController(Command):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        Robot.index.m_top.set_raw_output(
            Joysticks.joysticks[Keymap.Index.MANUAL_INDEX_CONTROLLER].getRawAxis(Keymap.Index.MANUAL_INDEX_TOP_AXIS) * 0.5
        )

        Robot.index.m_bottom.set_raw_output(
            Joysticks.joysticks[Keymap.Index.MANUAL_INDEX_CONTROLLER].getRawAxis(Keymap.Index.MANUAL_INDEX_BOTTOM_AXIS) * -0.5
        )

    def end(self, interrupted: bool) -> None:
        Robot.index.m_top.set_raw_output(0)
        Robot.index.m_bottom.set_raw_output(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
