from commands2 import InstantCommand
from robotpy_toolkit_7407.command import SubsystemCommand

from robot_systems import Robot
from subsystem import Index
from oi.keymap import Keymap


IndexOn = InstantCommand(lambda: Robot.index.set(.5), Robot.index)
IndexOff = InstantCommand(lambda: Robot.index.set(0), Robot.index)


class IndexDrive(SubsystemCommand[Index]):
    def __init__(self, subsystem: Index):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        left_joy = Keymap.Index.INDEX_JOY.value
        if abs(left_joy) < .1:
            self.subsystem.set(0)
        else:
            self.subsystem.set(-left_joy)

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
