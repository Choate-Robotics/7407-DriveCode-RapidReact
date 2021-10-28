import wpilib
import commands2 as commands

import utils.logger as logger
from robot_lib.subsystem import Subsystem


class Shifter(Subsystem):
    solenoid: wpilib.Solenoid

    def init(self) -> None:
        logger.info("initializing shifter", "[shifter]")

        # self.solenoid = wpilib.Solenoid(4)

        # self.solenoid.set(False)

        logger.info("initialization complete", "[shifter]")
