import time

from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveDrivetrain
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import bounded_angle_diff
from robotpy_toolkit_7407.utils.units import radians
from wpimath._controls._controls.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import TrapezoidProfileRadians
from wpimath.geometry import Rotation2d


class RotateInPlace(SubsystemCommand[SwerveDrivetrain]):
    def __init__(self, subsystem: SwerveDrivetrain, theta_f: radians, duration: float = 0.5, period: float = 0.02):
        super().__init__(subsystem)
        self.controller = HolonomicDriveController(
            PIDController(1, 0, 0, period),
            PIDController(1, 0, 0, period),
            ProfiledPIDControllerRadians(
                4, 0, 0, TrapezoidProfileRadians.Constraints(
                    subsystem.max_angular_vel,
                    subsystem.max_angular_vel / .01
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
        self.theta_i = self.subsystem.odometry.getPose().rotation().radians()
        self.theta_diff = bounded_angle_diff(self.theta_i, self.theta_f)
        self.omega = self.theta_diff / self.duration
        self.finished = False

    def execute(self) -> None:
        self.t = time.perf_counter() - self.start_time
        if self.t > self.duration:
            self.t = self.duration
            self.finished = True
        goal = self.subsystem.odometry.getPose()
        goal_theta = self.theta_i + self.omega * self.t
        speeds = self.controller.calculate(goal, goal, 0, Rotation2d(goal_theta))
        self.subsystem.set((0, 0), speeds.omega)

    def end(self, interrupted: bool) -> None:
        self.subsystem.set((0, 0), 0)

    def isFinished(self) -> bool:
        return self.finished

    def runsWhenDisabled(self) -> bool:
        return False
