import wpilib
import ctre
import commands2 as commands
import utils.logger as logger
from robot_lib.subsystem import Subsystem


class Turret(Subsystem):
    def init(self) -> None:
        logger.info("initializing turret", "[turret]")

        self.motor = ctre.TalonSRX(8)

        self.motor.setSelectedSensorPosition(0)

        self.motor.config_kP(0, 0.4)
        self.motor.config_kI(0, 0.0)
        self.motor.config_kD(0, 1.5)
        self.motor.configClosedLoopPeakOutput(0, 1)
        self.motor.configMotionCruiseVelocity(4000)
        self.motor.configMotionAcceleration(10000)

        logger.info("initialization complete", "[turret]")
