import math
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain
from robotpy_toolkit_7407.utils.units import m, s, rad
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from sensors.limelight import Limelight
from robot_systems import Robot
import constants


def curve_abs(x):
    return x ** 2


def curve(x):
    if x < 0:
        return -curve_abs(x)
    return curve_abs(x)


class DriveSwerveCustom(SubsystemCommand[SwerveDrivetrain]):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        dx, dy, d_theta = self.subsystem.axis_dx.value, self.subsystem.axis_dy.value, -self.subsystem.axis_rotation.value

        if abs(d_theta) < 0.15:
            d_theta = 0

        dx = curve(dx)
        dy = curve(dy)
        d_theta = curve(d_theta)

        # TODO normalize this to circle somehow
        dx *= self.subsystem.max_vel.asUnit(m/s)
        dy *= -self.subsystem.max_vel.asUnit(m/s)

        self.subsystem.set((dx, dy), d_theta * self.subsystem.max_angular_vel)

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_00.set(0 * m/s, 0 * rad)
        self.subsystem.n_01.set(0 * m/s, 0 * rad)
        self.subsystem.n_10.set(0 * m/s, 0 * rad)
        self.subsystem.n_11.set(0 * m/s, 0 * rad)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class DriveSwerveAim(SubsystemCommand[SwerveDrivetrain]):
    def initialize(self) -> None:
        Robot.limelight.ref_on()

    def execute(self) -> None:
        dx, dy = self.subsystem.axis_dx.value, self.subsystem.axis_dy.value
        #omega = self.controller.calculate(0, self.cam.get_x_offset()) * rad / s
        omega = -0.07 * Robot.limelight.get_x_offset() * rad / s #(The 3 is adjustable, p-gain)

        dx = curve(dx)
        dy = curve(dy)

        # TODO normalize this to circle somehow
        dx *= self.subsystem.max_vel.asUnit(m / s)
        dy *= -self.subsystem.max_vel.asUnit(m / s)

        if Robot.drivetrain.chassis_speeds.omega < .005 and Robot.limelight.get_x_offset() != 0:
            Robot.shooter.drive_ready = True
        else:
            Robot.shooter.drive_ready = False

        self.subsystem.set((dx, dy), omega)

    def end(self, interrupted: bool) -> None:
        Robot.shooter.drive_ready = False
        Robot.limelight.ref_off()

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
