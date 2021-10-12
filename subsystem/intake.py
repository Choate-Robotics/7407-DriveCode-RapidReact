import rev
import wpilib
import commands2 as commands

import utils.logger as logger


class Intake(commands.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        logger.info("initializing intake", "[intake]")

        self.motor = rev.CANSparkMax(12, rev.CANSparkMax.MotorType.kBrushless)
        self.solenoid = wpilib.DoubleSolenoid(0, 7)

        logger.info("initialization complete", "[intake]")
