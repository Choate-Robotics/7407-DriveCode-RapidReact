import math

from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain

from robot_systems import Robot


def curve_abs(x):
    return x ** 2.4


def curve(x):
    if x < 0:
        return -curve_abs(-x)
    return curve_abs(x)


class DriveSwerveCustom(SubsystemCommand[SwerveDrivetrain]):
    driver_centric = False

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
        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel

        if DriveSwerveCustom.driver_centric:
            self.subsystem.set_driver_centric((dy, -dx), d_theta * self.subsystem.max_angular_vel)
        else:
            self.subsystem.set((dx, dy), d_theta * self.subsystem.max_angular_vel)

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_00.set(0, 0)
        self.subsystem.n_01.set(0, 0)
        self.subsystem.n_10.set(0, 0)
        self.subsystem.n_11.set(0, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class DriveSwerveAim(SubsystemCommand[SwerveDrivetrain]):
    def __init__(self, subsystem, ready_counts=2):
        super().__init__(subsystem)
        self.ready_counts = ready_counts
        self.c_count = 0
        self.KP = -0.06
        self.KD = .040
        self.old_limelight = 0

    def initialize(self) -> None:
        Robot.limelight.ref_on()
        self.old_limelight = Robot.limelight.get_x_offset()

    def execute(self) -> None:
        dx, dy = self.subsystem.axis_dx.value, self.subsystem.axis_dy.value
        d_omega = math.degrees(self.old_limelight - Robot.limelight.get_x_offset())
        omega = self.KP * math.degrees(Robot.limelight.get_x_offset()) + self.KD * d_omega
        self.old_limelight = Robot.limelight.get_x_offset()

        dx = curve(dx)
        dy = curve(dy)

        # TODO normalize this to circle somehow
        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel

        if abs(Robot.drivetrain.chassis_speeds.omega) < .1 and Robot.limelight.get_x_offset() != 0:
            self.c_count += 1
            if self.c_count >= self.ready_counts:
                Robot.shooter.drive_ready = True
        else:
            self.c_count = 0
            Robot.shooter.drive_ready = False

        self.subsystem.set((dx, dy), omega)

    def end(self, interrupted: bool) -> None:
        Robot.shooter.drive_ready = False
        Robot.limelight.ref_off()

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
