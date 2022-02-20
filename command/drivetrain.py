import math

from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain
from robotpy_toolkit_7407.utils.units import m, s


class DriveSwerveCustom(SubsystemCommand[SwerveDrivetrain]):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        dx, dy, d_theta = self.subsystem.axis_dx.value, self.subsystem.axis_dy.value, self.subsystem.axis_rotation.value

        def curve_abs(x):
            return x ** 2

        def curve(x):
            if x < 0:
                return -curve_abs(x)
            return curve_abs(x)

        dx = curve(dx)
        dy = curve(dy)

        # TODO normalize this to circle somehow
        dx *= self.subsystem.max_vel.asUnit(m/s)
        dy *= -self.subsystem.max_vel.asUnit(m/s)

        self.subsystem.set((dx, dy), d_theta * self.subsystem.max_angular_vel)

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
