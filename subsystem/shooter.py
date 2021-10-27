import wpilib
import ctre
import commands2 as commands
import utils.logger as logger
from robot_lib.motor import PIDMotor
from robot_lib.motors.ctre_motors import TalonFX, TalonGroup, TalonConfig
from robot_lib.subsystem import Subsystem


class Shooter(Subsystem):
    _config: TalonConfig = TalonConfig(k_F=1023.0/19930.0, neutral_brake=True)
    motor: PIDMotor = TalonGroup(TalonFX(6, False), TalonFX(7, True), config=_config)
    hood: wpilib.DoubleSolenoid
    shooter_settings: tuple[float, bool]

    def init(self) -> None:
        logger.info("initializing shooter", "[shooter]")
        
        self.motor.init()

        self.hood = wpilib.DoubleSolenoid(1, 6)

        self.shooter_settings = (0.0, False)

        logger.info("initialization complete", "[shooter]")
