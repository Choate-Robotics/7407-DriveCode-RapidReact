from typing import Tuple
import commands2 as commands
import ctre

import subsystem
from oi import OI
from oi.keymap import Keymap
from utils.units import sensor_units_to_inches, inches_to_sensor_units


class DriveArcade(commands.CommandBase):
    def __init__(self, drivetrain: subsystem.Drivetrain, oi: OI) -> None:
        super().__init__()
        self.addRequirements(drivetrain)
        self._drivetrain = drivetrain
        self._oi = oi

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        x_axis = self._oi.joysticks[Keymap.Drivetrain.DRIVE_CONTROLLER].getRawAxis(Keymap.Drivetrain.DRIVE_X_AXIS)
        y_axis = self._oi.joysticks[Keymap.Drivetrain.DRIVE_CONTROLLER].getRawAxis(Keymap.Drivetrain.DRIVE_Y_AXIS)

        x_axis, y_axis = self._add_dead_zones(x_axis, y_axis)
        left, right = self._turn_radius_drive(x_axis, y_axis)

        self._drivetrain.left1.set(ctre.ControlMode.Velocity, left)
        self._drivetrain.right1.set(ctre.ControlMode.Velocity, right)

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False

    @staticmethod
    def _add_dead_zones(x_axis: float, y_axis: float) -> Tuple[float, float]:
        if x_axis < 0.2:
            x_axis = 0
        if y_axis < 0.2:
            y_axis = 0

        return x_axis, y_axis

    @staticmethod
    def _turn_radius_drive(x_axis: float, y_axis: float) -> Tuple[float, float]:
        if y_axis > 0:
            x_axis = -x_axis

        x_axis *= abs(x_axis)

        velocity_sensor_units = 18000 * y_axis
        target_velocity = sensor_units_to_inches(velocity_sensor_units, True)

        distance_between_wheels = 25.111

        if x_axis > 0:
            turn_radius = 120.0 * (1.05 - x_axis)
        elif x_axis < 0:
            turn_radius = 120.0 * (-1.05 - x_axis)
        else:
            turn_radius = 0

        if x_axis == 0:
            velocity_difference = 0
        elif target_velocity == 0 or turn_radius == 0:
            velocity_difference = sensor_units_to_inches(-18000.0 * x_axis, True)
        else:
            velocity_difference = (distance_between_wheels * target_velocity) / turn_radius

        left = inches_to_sensor_units(target_velocity - velocity_difference, True)
        right = inches_to_sensor_units(target_velocity + velocity_difference, True)

        return left, right
