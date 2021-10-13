import wpilib
import ctre
import commands2 as commands
import utils.logger as logger
from robot_lib.subsystem import Subsystem


class Shooter(Subsystem):
    def init(self) -> None:
        logger.info("initializing shooter", "[shooter]")
        
        self.shooter1 = ctre.TalonFX(6)
        self.shooter2 = ctre.TalonFX(7)

        self.shooter1.setNeutralMode(ctre.NeutralMode.Brake)
        self.shooter2.setNeutralMode(ctre.NeutralMode.Brake)

        self.shooter1.config_kF(0, 1023.0 / 19930.0)
        self.shooter2.config_kF(0, 1023.0 / 19930.0)

        self.shooter2.follow(self.shooter1)
        self.shooter2.setInverted(True)

        self.hood = wpilib.DoubleSolenoid(1, 6)

        self.shooter_settings = (0.0, False)

        logger.info("initialization complete", "[shooter]")
