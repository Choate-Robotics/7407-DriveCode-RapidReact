import math

from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain


class DriveSwerveCustom(SubsystemCommand[SwerveDrivetrain]):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        dx, dy, d_theta = self.subsystem.axis_dx.value, self.subsystem.axis_dy.value, self.subsystem.axis_rotation.value

        # if abs(dx * dx + dy * dy) < 0.00025:
        #     dx = 0
        #     dy = 0
        if abs(d_theta) < 0.1:
            d_theta = 0

        def curve_abs(x):
            return x ** 2

        def curve(x):
            if x < 0:
                return -curve_abs(x)
            return curve_abs(x)

        dx = curve(dx)
        dy = curve(dy)

        dx *= 4
        dy *= -4
        d_theta *= -4 * math.pi / 3

        self.subsystem.set((dx, dy), d_theta)

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False