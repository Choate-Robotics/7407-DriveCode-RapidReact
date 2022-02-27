import time

from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveDrivetrain
from robotpy_toolkit_7407.unum import Unum
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import bounded_angle_diff, rotate_vector
from robotpy_toolkit_7407.utils.units import rad, s, m
from wpimath.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians
from wpimath.geometry import Rotation2d


class FollowPathCustom(SubsystemCommand[SwerveDrivetrain]):
    def __init__(self, subsystem: SwerveDrivetrain, trajectory: Trajectory, theta_f: Unum, period: float = 0.02):
        super().__init__(subsystem)
        self.trajectory = trajectory
        self.controller = HolonomicDriveController(
            PIDController(1, 0, 0, period),
            PIDController(1, 0, 0, period),
            ProfiledPIDControllerRadians(
                4, 0, 0, TrapezoidProfileRadians.Constraints(
                    subsystem.max_angular_vel.asNumber(rad/s),
                    (subsystem.max_angular_vel / (.01 * s)).asNumber(rad/(s**2))
                ), period
            )
        )
        self.start_time = 0
        self.t = 0
        self.duration = trajectory.totalTime()
        self.theta_f = theta_f
        self.theta_i: float | None = None
        self.theta_diff: float | None = None
        self.omega: float | None = None
        self.finished: bool = False

    def initialize(self) -> None:
        self.start_time = time.perf_counter()
        self.theta_i = self.subsystem.odometry.getPose().rotation().radians() * rad
        self.theta_diff = bounded_angle_diff(self.theta_i.asNumber(rad), self.theta_f.asNumber(rad)) * rad
        self.omega = self.theta_diff / (self.duration * s)
        self.finished = False

    def execute(self) -> None:
        self.t = time.perf_counter() - self.start_time
        if self.t > self.duration:
            self.t = self.duration
            self.finished = True
        goal = self.trajectory.sample(self.t)
        goal_theta = self.theta_i + self.omega * self.t * s
        speeds = self.controller.calculate(self.subsystem.odometry.getPose(), goal, Rotation2d(goal_theta.asNumber(rad)))
        vx, vy = rotate_vector(
            speeds.vx * m/s, speeds.vy * m/s,
            self.subsystem.odometry.getPose().rotation().radians() * rad
        )
        self.subsystem.set((vx, vy), speeds.omega * rad/s)
        # self.subsystem.set((vx, vy), 0 * rad/s)

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return self.finished

    def runsWhenDisabled(self) -> bool:
        return False


class RotateInPlace(SubsystemCommand[SwerveDrivetrain]):
    def __init__(self, subsystem: SwerveDrivetrain, theta_f: Unum, duration: float = 0.5, period: float = 0.02):
        super().__init__(subsystem)
        self.controller = HolonomicDriveController(
            PIDController(1, 0, 0, period),
            PIDController(1, 0, 0, period),
            ProfiledPIDControllerRadians(
                4, 0, 0, TrapezoidProfileRadians.Constraints(
                    subsystem.max_angular_vel.asNumber(rad/s),
                    (subsystem.max_angular_vel / (.01 * s)).asNumber(rad/(s**2))
                ), period
            )
        )
        self.start_time = 0
        self.t = 0
        self.duration = duration
        self.theta_f = theta_f
        self.theta_i: float | None = None
        self.theta_diff: float | None = None
        self.omega: float | None = None
        self.finished = False

    def initialize(self) -> None:
        logger.info(f"rotating")
        self.start_time = time.perf_counter()
        self.theta_i = self.subsystem.odometry.getPose().rotation().radians() * rad
        self.theta_diff = bounded_angle_diff(self.theta_i.asNumber(rad), self.theta_f.asNumber(rad)) * rad
        self.omega = self.theta_diff / (self.duration * s)
        self.finished = False

    def execute(self) -> None:
        self.t = time.perf_counter() - self.start_time
        if self.t > self.duration:
            self.t = self.duration
            self.finished = True
        goal = self.subsystem.odometry.getPose()
        goal_theta = self.theta_i + self.omega * self.t * s
        speeds = self.controller.calculate(goal, goal, 0, Rotation2d(goal_theta.asNumber(rad)))
        self.subsystem.set((0 * m/s, 0 * m/s), speeds.omega * rad/s)

    def end(self, interrupted: bool) -> None:
        self.subsystem.set((0 * m/s, 0 * m/s), 0 * rad/s)

    def isFinished(self) -> bool:
        return self.finished

    def runsWhenDisabled(self) -> bool:
        return False
