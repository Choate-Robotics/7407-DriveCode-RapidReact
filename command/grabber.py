from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Grabber


class GrabberMoveControl(SubsystemCommand[Grabber]):
    def __init__(self, subsystem: Grabber, down: bool = True):
        super().__init__(subsystem)
        self.down = down

    def initialize(self) -> None:
        self.subsystem.set_move(self.down)

    def isFinished(self) -> bool:
        return True


class GrabberGrabControl(SubsystemCommand[Grabber]):
    def __init__(self, subsystem: Grabber, on: bool = True):
        super().__init__(subsystem)
        self.on = on

    def initialize(self) -> None:
        self.subsystem.set_grab(self.on)

    def isFinished(self) -> bool:
        return True

