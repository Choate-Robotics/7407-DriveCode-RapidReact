import ctre
import commands2 as commands

import subsystem


class IndexShoot(commands.CommandBase):
    def __init__(self, index: subsystem.Index) -> None:
        super().__init__()
        self.addRequirements(index)
        self._index = index

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self._index.top_motor.set(-0.3)
        self._index.bottom_motor.set(ctre.ControlMode.PercentOutput, 0.5)

    def end(self, interrupted: bool) -> None:
        self._index.top_motor.set(0)
        self._index.bottom_motor.set(ctre.ControlMode.PercentOutput, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
