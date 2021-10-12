import ctre
import commands2 as commands

import subsystem
from oi import OI
from oi.keymap import Keymap


class IndexManualSpeedController(commands.CommandBase):
    def __init__(self, index: subsystem.Index, oi: OI) -> None:
        super().__init__()
        self.addRequirements(index)
        self._index = index
        self._oi = oi

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self._index.top_motor.set(
            self._oi.joysticks[Keymap.Index.MANUAL_INDEX_CONTROLLER]
                .getRawAxis(Keymap.Index.MANUAL_INDEX_TOP_AXIS) * 0.5
        )

        self._index.bottom_motor.set(
            ctre.ControlMode.PercentOutput,
            self._oi.joysticks[Keymap.Index.MANUAL_INDEX_CONTROLLER]
                .getRawAxis(Keymap.Index.MANUAL_INDEX_BOTTOM_AXIS) * -0.5
        )

    def end(self, interrupted: bool) -> None:
        self._index.top_motor.set(0)
        self._index.bottom_motor.set(ctre.ControlMode.PercentOutput, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
