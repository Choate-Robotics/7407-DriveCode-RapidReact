import rev
import ctre
import wpilib
import commands2 as commands

import utils.logger as logger
from robot_lib.motor import Motor
from robot_lib.motors.ctre_motors import VictorSPX
from robot_lib.motors.rev_motors import SparkMax
from robot_lib.subsystem import Subsystem


class Index(Subsystem):
    m_top: Motor = SparkMax(10)
    m_bottom: Motor = VictorSPX(11)
    solenoid: wpilib.DoubleSolenoid

    def init(self) -> None:
        logger.info("initializing index", "[index]")

        self.m_top.init()
        self.m_bottom.init()

        self.solenoid = wpilib.DoubleSolenoid(2, 5)

        logger.info("initialization complete", "[index]")
