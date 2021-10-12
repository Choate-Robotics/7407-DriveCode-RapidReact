import ctre
import commands2 as commands

import utils.logger as logger


class Hopper(commands.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        logger.info("initializing hopper", "[hopper]")

        self.motor = ctre.VictorSPX(13)

        logger.info("initialization complete", "[hopper]")
