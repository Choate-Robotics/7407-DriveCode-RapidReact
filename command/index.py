
from commands2 import InstantCommand
from robotpy_toolkit_7407.command import SubsystemCommand

from robot_systems import Robot
from subsystem import Index
from oi.keymap import Keymap



IndexOn = lambda: InstantCommand(lambda: Robot.index.set(.5), Robot.index)
IndexOff = lambda: InstantCommand(lambda: Robot.index.set(0), Robot.index)

class IndexAutoDrive(SubsystemCommand):
    def __init__(self, subsystem: Index):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        pass
    def execute(self):
        speed = 0
        if Robot.intake.left_dinglebob_in and Robot.intake.right_dinglebob_in:
            speed = .5

        if self.subsystem.photo_electric.get_value():
            speed = 0

        left_joy = Keymap.Index.INDEX_JOY.value
        if abs(left_joy) < .1:
            pass
        else:
            if left_joy < 0:
                speed = .5
            else:
                speed = -.5

        self.subsystem.set(speed)

    def isFinished(self) -> bool:
        return False
    def end(self, interrupted=False):
        pass

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
                self.was_on = False
        else:
            if left_joy < 0:
                self.subsystem.set(.5)
            else:
                self.subsystem.set(-.5)
            self.was_on = True

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False