import math
import time

from robotpy_toolkit_7407.command import SubsystemCommand, Command
from robotpy_toolkit_7407.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians

import constants
from robot_systems import Robot
from subsystem import Drivetrain, Shooter


def curve_abs(x):
    return x ** 2.4


def curve(x):
    if x < 0:
        return -curve_abs(-x)
    return curve_abs(x)


AIM_kP = 3.5
AIM_kI = 0
AIM_kD = 0
AIM_max_angular_vel = 6
AIM_max_angular_accel = 6


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


class ShootWhileMoving(Command):
    def __init__(self, drivetrain: Drivetrain, shooter: Shooter):
        super().__init__()

        self.drivetrain = drivetrain
        self.shooter = shooter

        self.addRequirements(drivetrain, shooter)
        self.pid_controller = ProfiledPIDControllerRadians(
            AIM_kP, AIM_kI, AIM_kD,
            TrapezoidProfileRadians.Constraints(
                AIM_max_angular_vel, AIM_max_angular_accel
            ), constants.period
        )

        self.should_shoot_time = None
        self.shoot_vel = None

    def initialize(self) -> None:
        self.should_shoot_time = None
        self.shoot_vel = None
        self.drivetrain.max_vel = constants.drivetrain_target_max_vel

    def execute(self) -> None:
        hub_angle = Robot.odometry.hub_angle
        dx, dy = self.drivetrain.axis_dx.value, self.drivetrain.axis_dy.value

        robot_dist = Robot.odometry.hub_dist
        if robot_dist is None:
            robot_dist = 2

        robot_vel = Robot.drivetrain.chassis_speeds.vx, Robot.drivetrain.chassis_speeds.vy

        target_angle, should_shoot = self.shooter.target_with_motion(robot_dist, hub_angle, robot_vel)

        omega = self.pid_controller.calculate(target_angle, 0)

        dx = curve(dx)
        dy = curve(dy)

        # TODO normalize this to circle somehow
        dx *= self.drivetrain.max_vel
        dy *= -self.drivetrain.max_vel

        if should_shoot:
            Robot.shooter.ready = True
            self.shoot_vel = robot_vel
            self.should_shoot_time = time.time()
        elif self.should_shoot_time is None or self.should_shoot_time + 0.3 < time.time():
            Robot.shooter.ready = False

        if Robot.shooter.ready:
            self.drivetrain.set_driver_centric(self.shoot_vel, omega)
        else:
            self.drivetrain.set((dx, dy), omega)

    def end(self, interrupted: bool) -> None:
        Robot.shooter.ready = False
        Robot.shooter.stop()
        self.drivetrain.max_vel = constants.drivetrain_max_vel

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
