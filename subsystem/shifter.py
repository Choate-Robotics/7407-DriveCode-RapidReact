import wpilib
import commands2 as commands

import utils.logger as logger


class Shifter(commands.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        logger.info("initializing shifter", "[shifter]")

        self.solenoid = wpilib.Solenoid(4)

        self.solenoid.set(False)

        logger.info("initialization complete", "[shifter]")
