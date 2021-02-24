import commands2 as commands

import subsystem


class IntakeMotorControlCommand(commands.CommandBase):
    MOTOR_SPEED: float

    def __init__(self, intake: subsystem.Intake) -> None:
        super().__init__()
        self.addRequirements(intake)
        self._intake = intake

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self._intake.motor.set(self.MOTOR_SPEED)

    def end(self, interrupted: bool) -> None:
        self._intake.motor.set(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class IntakeRun(IntakeMotorControlCommand):
    MOTOR_SPEED = -0.95


class IntakeRunReverse(IntakeMotorControlCommand):
    MOTOR_SPEED = 1.0
