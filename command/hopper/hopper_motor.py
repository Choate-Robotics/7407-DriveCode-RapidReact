import ctre
import commands2 as commands

import subsystem


class HopperRun(commands.CommandBase):
    def __init__(self, hopper: subsystem.Hopper) -> None:
        super().__init__()
        self.addRequirements(hopper)
        self._hopper = hopper

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self._hopper.motor.set(ctre.ControlMode.PercentOutput, -0.5)

    def end(self, interrupted: bool) -> None:
        self._hopper.motor.set(ctre.ControlMode.PercentOutput, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class HopperRunReverse(commands.CommandBase):
    def __init__(self, hopper: subsystem.Hopper) -> None:
        super().__init__()
        self.addRequirements(hopper)
        self._hopper = hopper

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self._hopper.motor.set(ctre.ControlMode.PercentOutput, 0.5)

    def end(self, interrupted: bool) -> None:
        self._hopper.motor.set(ctre.ControlMode.PercentOutput, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
