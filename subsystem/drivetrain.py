from lib.motor import PIDMotor
from lib.motors.ctre_motors import TalonConfig, TalonGroup, TalonFX
from lib.subsystem_templates.drivetrain.differential_drivetrain import DifferentialDrivetrain


_MOTOR_CONFIG = TalonConfig(0.5, 0, 0, 1, 15000, 10000)


class Drivetrain(DifferentialDrivetrain):
    m_left: PIDMotor = TalonGroup(TalonFX(0), TalonFX(1), TalonFX(2), config=_MOTOR_CONFIG)
    m_right: PIDMotor = TalonGroup(TalonFX(3), TalonFX(4), TalonFX(5), config=_MOTOR_CONFIG)
