from telnetlib import EL
from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Elevator


class ElevatorUp(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: Elevator) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.up()

    def isFinished(self) -> bool:
        return True


class ElevatorDown(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: Elevator) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.down()

    def isFinished(self) -> bool:
        return True


class ElevatorStop(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: Elevator) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.stop()

    def isFinished(self) -> bool:
        return True


class ElevatorSolenoidToggle(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: Elevator) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.toggle_elevator()

    def isFinished(self) -> bool:
        return True
