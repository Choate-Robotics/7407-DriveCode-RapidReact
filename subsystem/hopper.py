import ctre
import commands2 as commands

import utils.logger as logger
from robot_lib.subsystem import Subsystem


class Hopper(Subsystem):
    def init(self) -> None:
        logger.info("initializing hopper", "[hopper]")

        self.motor = ctre.VictorSPX(13)

        logger.info("initialization complete", "[hopper]")
