import utils.logger as logger
from robot_lib.motor import PIDMotor
from robot_lib.motors.ctre_motors import TalonSRX, TalonConfig
from robot_lib.subsystem import Subsystem


class Turret(Subsystem):
    _config: TalonConfig = TalonConfig(0.4, 0, 1.5, None, 1, 4000, 10000)
    motor: PIDMotor = TalonSRX(8)

    def init(self) -> None:
        logger.info("initializing turret", "[turret]")

        self.motor.init()

        logger.info("initialization complete", "[turret]")
