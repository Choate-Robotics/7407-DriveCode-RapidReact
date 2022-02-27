from commands2 import InstantCommand
from robotpy_toolkit_7407.command import SubsystemCommand

from robot_systems import Robot
from subsystem import Index
from oi.keymap import Keymap


IndexOn = lambda: InstantCommand(lambda: Robot.index.set(.5), Robot.index)
IndexOff = lambda: InstantCommand(lambda: Robot.index.set(0), Robot.index)


class IndexDrive(SubsystemCommand[Index]):
    def __init__(self, subsystem: Index):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.was_on = False

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        left_joy = Keymap.Index.INDEX_JOY.value
        if abs(left_joy) < .1:
            self.subsystem.set(0)
            if self.was_on:
                Robot.intake.m_top.set_raw_output(0)
                self.was_on = False
        else:
            if left_joy < 0:
                self.subsystem.set(.5)
            else:
                self.subsystem.set(-.5)
            self.was_on = True
            Robot.intake.m_top.set_raw_output(0.7)  # TODO bad thing

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
