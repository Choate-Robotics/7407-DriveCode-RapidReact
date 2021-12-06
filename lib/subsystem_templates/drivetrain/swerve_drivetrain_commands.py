import math

from lib.command import SubsystemCommand
from lib.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain


class DriveSwerve(SubsystemCommand[SwerveDrivetrain]):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        dx, dy, d_theta = self.subsystem.axis_dx.value, self.subsystem.axis_dy.value, self.subsystem.axis_rotation.value

        dx *= -4
        dy *= 4
        d_theta *= -4 * math.pi / 3

        self.subsystem.set((dx, dy), d_theta)

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()

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
