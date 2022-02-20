from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Intake


class IntakeOn(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.set(.5)

    def isFinished(self) -> bool:
        return True

class IntakeOff(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.set(0)

    def isFinished(self) -> bool:
        return True

class IntakeLeftSolenoidToggle(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.toggleLeftIntake()

    def isFinished(self) -> bool:
        return True

class IntakeRightSolenoidToggle(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake) -> None:  
        super().__init__(subsystem)
    
    def execute(self) -> None:
        self.subsystem.toggleRightIntake()

    def isFinished(self) -> bool:
        return True