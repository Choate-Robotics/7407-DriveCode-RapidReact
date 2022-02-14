from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Elevator


class ElevatorUp(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: Elevator) -> None:  
        super().__init__(subsystem)

    def initialize(self) -> None:
        self.subsystem.alter_left_speed(self.change)
    
    def execute(self) -> None:
        Elevator.up()

    def isFinished(self) -> bool:
        return True

class ElevatorDown(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: Elevator) -> None:  
        super().__init__(subsystem)

    def initialize(self) -> None:
        self.subsystem.alter_left_speed(self.change)
    
    def execute(self) -> None:
        Elevator.down()

    def isFinished(self) -> bool:
        return True