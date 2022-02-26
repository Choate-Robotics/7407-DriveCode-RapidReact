from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit

from robot_systems import Robot
from subsystem import Shooter
from robotpy_toolkit_7407.utils.units import m, s

class ShooterEnable(SubsystemCommand[Shooter]):
    def initialize(self) -> None:
        self.subsystem.target(Robot.limelight.calculate_distance())

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()

class ShooterZero(SubsystemCommand[Shooter]):
    def initialize(self, subsystem) -> None:
        self.subsystem = subsystem
        self.subsystem.m_angle.set_target_velocity(-.1 * m/s)
    def execute(self):
        if self.subsystem.left_limit.get_value():
            self.subsystem.zeroed = True
    def isFinished(self) -> bool:
        return self.subsystem.zeroed
    def end(self) -> None:
        self.subsystem.stop()
        self.subsystem.m_angle.set_sensor_position(0 * talon_sensor_unit)
