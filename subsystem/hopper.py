import ctre
import commands2 as commands

import utils.logger as logger
from robot_lib.motor import Motor
from robot_lib.motors.ctre_motors import VictorSPX
from robot_lib.subsystem import Subsystem


class Hopper(Subsystem):
    motor: Motor = VictorSPX(13)

    def init(self) -> None:
        logger.info("initializing hopper", "[hopper]")

        self.motor.init()

        logger.info("initialization complete", "[hopper]")
