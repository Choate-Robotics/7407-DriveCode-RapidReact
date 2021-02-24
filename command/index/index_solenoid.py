import wpilib
import commands2 as commands

import subsystem


class IndexSolenoidControlCommand(commands.CommandBase):
    SOLENOID_STATE: wpilib.DoubleSolenoid.Value

    def __init__(self, index: subsystem.Index) -> None:
        super().__init__()
        self.addRequirements(index)
        self._index = index

    def initialize(self) -> None:
        self._index.solenoid.set(self.SOLENOID_STATE)

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return False


class IndexUp(IndexSolenoidControlCommand):
    SOLENOID_STATE = wpilib.DoubleSolenoid.Value.kForward


class IndexDown(IndexSolenoidControlCommand):
    SOLENOID_STATE = wpilib.DoubleSolenoid.Value.kReverse
