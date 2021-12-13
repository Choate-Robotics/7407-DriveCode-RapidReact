import constants
from lib.command import SubsystemCommand
from lib.subsystem_templates.drivetrain.differential_drivetrain import DifferentialDrivetrain
from utils.math import clamp, sensor_units_to_inches, inches_to_sensor_units


class DriveArcade(SubsystemCommand[DifferentialDrivetrain]):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        x_axis, y_axis = self.subsystem.axis_x.value, self.subsystem.axis_y.value

        x_axis, y_axis = self._add_dead_zones(x_axis, y_axis)

        left, right = self._turn_radius_drive(x_axis, y_axis)

        self.subsystem.set_motor_velocity(left, -right)

    def end(self, interrupted: bool) -> None:
        self.subsystem.set_motor_velocity(0, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False

    @staticmethod
    def _add_dead_zones(x_axis: float, y_axis: float) -> tuple[float, float]:
        if abs(x_axis) < 0.2:
            x_axis = 0
        if abs(y_axis) < 0.2:
            y_axis = 0

        return x_axis, y_axis

    @staticmethod
    def _arcade_drive(x_axis: float, y_axis: float) -> tuple[float, float]:
        left = clamp(y_axis + x_axis, -1, 1)
        right = clamp(y_axis - x_axis, -1, 1)

        # left *= 18000
        # right *= 18000

        return left, right

    @staticmethod
    def _turn_radius_drive(x_axis: float, y_axis: float) -> tuple[float, float]:
        if y_axis > 0:
            x_axis = -x_axis

        # x_axis *= abs(x_axis)

        velocity_sensor_units = 18000 * y_axis
        target_velocity = sensor_units_to_inches(velocity_sensor_units, True)

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
            velocity_difference = (constants.track_width_inches * target_velocity) / turn_radius

        left = inches_to_sensor_units(target_velocity - velocity_difference, True)
        right = inches_to_sensor_units(target_velocity + velocity_difference, True)

        left = clamp(left, -18000, 18000)
        right = clamp(right, -18000, 18000)

        return left, right