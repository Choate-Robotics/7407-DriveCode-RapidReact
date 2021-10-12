import wpilib
import commands2 as commands

import subsystem


class IntakeSolenoidControlCommand(commands.CommandBase):
    SOLENOID_STATE: wpilib.DoubleSolenoid.Value

    def __init__(self, intake: subsystem.Intake) -> None:
        super().__init__()
        self.addRequirements(intake)
        self._intake = intake

    def initialize(self) -> None:
        self._intake.solenoid.set(self.SOLENOID_STATE)

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return False


class IntakeUp(IntakeSolenoidControlCommand):
    SOLENOID_STATE = wpilib.DoubleSolenoid.Value.kForward


class IntakeDown(IntakeSolenoidControlCommand):
    SOLENOID_STATE = wpilib.DoubleSolenoid.Value.kReverse
