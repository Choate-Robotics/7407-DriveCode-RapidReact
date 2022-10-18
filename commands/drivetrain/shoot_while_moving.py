import time

import commands2
from robotpy_toolkit_7407 import Command
from wpimath._controls._controls.controller import ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import TrapezoidProfileRadians

import constants
from commands.drivetrain.constants import AIM_kP, AIM_kI, AIM_kD, AIM_max_angular_vel, AIM_max_angular_accel
from commands.drivetrain.util import curve
from commands.drivetrain.drive_swerve_custom import DriveSwerveCustom
from robot_systems import Robot
from subsystem import Drivetrain, Shooter


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
        self.shooter.shooting_over = False

    def initialize(self) -> None:
        self.should_shoot_time = None
        self.shoot_vel = None
        self.shooter.shooting_over = False

    def execute(self) -> None:
        hub_angle = Robot.odometry.hub_angle
        dx, dy = self.drivetrain.axis_dx.value, self.drivetrain.axis_dy.value

        robot_dist = Robot.odometry.hub_dist
        if robot_dist is None:
            robot_dist = 2

        robot_vel = Robot.drivetrain.chassis_speeds.vx, Robot.drivetrain.chassis_speeds.vy

        target_angle, should_shoot = self.shooter.target_with_motion(robot_dist - 0.2, hub_angle, robot_vel)

        omega = self.pid_controller.calculate(target_angle, 0)

        dx = curve(dx)
        dy = curve(dy)

        # TODO normalize this to circle somehow
        dx *= constants.drivetrain_target_max_vel
        dy *= -constants.drivetrain_target_max_vel

        if should_shoot and not self.shooter.shooting_over:
            self.shooter.ready = True
            self.shoot_vel = robot_vel
            self.should_shoot_time = time.time()
        # elif self.should_shoot_time is None or self.should_shoot_time + 0.3 < time.time():
        #     self.shooter.ready = False
        else:
            self.shooter.ready = False

        if self.shooter.ready:
            self.drivetrain.set_driver_centric(self.shoot_vel, omega)
        else:
            self.drivetrain.set((dx, dy), omega)

    def end(self, interrupted: bool) -> None:
        self.shooter.ready = False
        self.shooter.stop()

    def isFinished(self) -> bool:
        finished = not self.shooter.ready and self.shooter.shooting_over
        if finished:
            commands2.CommandScheduler.getInstance().schedule(DriveSwerveCustom(Robot.drivetrain))
            Robot.shooter.ready = False
            if not Robot.index.photo_electric.get_value():
                Robot.index.ball_queue = 0
                Robot.index.refresh = True
        return finished

    def runsWhenDisabled(self) -> bool:
        return False
