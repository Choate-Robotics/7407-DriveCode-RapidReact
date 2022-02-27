from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit
from robotpy_toolkit_7407.utils import logger

from robot_systems import Robot
from subsystem import Shooter
from robotpy_toolkit_7407.utils.units import m, s, rad


class ShooterEnable(SubsystemCommand[Shooter]):
    def initialize(self) -> None:
        # self.subsystem.target(Robot.limelight.calculate_distance())
        d = Robot.limelight.calculate_distance()
        self.subsystem.target_stationary(Robot.limelight.calculate_distance())
        logger.info(f"distance = {d}")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()


class ShooterZero(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.m_angle.set_raw_output(-0.06)

    def execute(self):
        if self.subsystem.left_limit.get_value():
            self.subsystem.zeroed = True

    def isFinished(self) -> bool:
        return self.subsystem.zeroed

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()
        self.subsystem.m_angle.set_sensor_position(0 * talon_sensor_unit)
