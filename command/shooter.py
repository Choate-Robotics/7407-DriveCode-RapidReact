from re import sub
from commands2 import InstantCommand
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit
from robotpy_toolkit_7407.unum import Unum
from robotpy_toolkit_7407.utils import logger

from robot_systems import Robot
from subsystem import Shooter
from robotpy_toolkit_7407.utils.units import m, s, rad, Unum
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_vel_unit

def change_offset(change: float):
    Robot.shooter.offset_m += change
    logger.info(f"NEW OFFSET = {Robot.shooter.offset_m}m")


ShooterOffsetUp = lambda: InstantCommand(lambda: change_offset(0.1))
ShooterOffsetDown = lambda: InstantCommand(lambda: change_offset(-0.1))


class ShooterEnable(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        Robot.limelight.ref_on()

    def execute(self) -> None:
        self.subsystem.target_stationary(Robot.limelight.calculate_distance() + Robot.shooter.offset_m)
    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()
        Robot.limelight.ref_off()


class ShooterEnableAtDistance(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter, distance: float):
        super().__init__(subsystem)
        self.distance = distance

    def initialize(self) -> None:
        self.subsystem.target_stationary(self.distance)

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
