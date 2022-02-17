from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Intake


class IndexOn(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.set(.45)

    def isFinished(self) -> bool:
        return True

class IndexOff(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.set(0)

    def isFinished(self) -> bool:
        return True