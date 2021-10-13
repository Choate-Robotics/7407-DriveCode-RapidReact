import rev
import ctre
import wpilib
import commands2 as commands

import utils.logger as logger
from robot_lib.subsystem import Subsystem


class Index(Subsystem):
    def init(self) -> None:
        logger.info("initializing index", "[index]")

        self.top_motor = rev.CANSparkMax(10, rev.CANSparkMax.MotorType.kBrushless)
        self.bottom_motor = ctre.VictorSPX(11)

        self.solenoid = wpilib.DoubleSolenoid(2, 5)

        logger.info("initialization complete", "[index]")
