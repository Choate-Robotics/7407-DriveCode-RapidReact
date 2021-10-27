import utils.logger as logger
from robot_lib.motor import PIDMotor
from robot_lib.motors.ctre_motors import VictorSPX
from robot_lib.subsystem import Subsystem


class Hanger(Subsystem):
    motor: PIDMotor = VictorSPX(14)

    def init(self) -> None:
        logger.info("initializing hanger", "[hanger]")

        self.motor.init()

        logger.info("initialization complete", "[hanger]")
