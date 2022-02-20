from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Index


class IndexOn(SubsystemCommand[Index]):
    def __init__(self, subsystem: Index) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.set(.5)

    def isFinished(self) -> bool:
        return True


class IndexOff(SubsystemCommand[Index]):
    def __init__(self, subsystem: Index) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.set(0)

    def isFinished(self) -> bool:
        return True
