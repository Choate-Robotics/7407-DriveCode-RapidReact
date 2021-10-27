import rev
import wpilib
import commands2 as commands

import utils.logger as logger
from robot_lib.motor import Motor
from robot_lib.motors.rev_motors import SparkMax
from robot_lib.subsystem import Subsystem


class Intake(Subsystem):
    motor: Motor = SparkMax(12)
    solenoid: wpilib.DoubleSolenoid

    def init(self) -> None:
        logger.info("initializing intake", "[intake]")

        self.motor.init()
        self.solenoid = wpilib.DoubleSolenoid(0, 7)

        logger.info("initialization complete", "[intake]")
