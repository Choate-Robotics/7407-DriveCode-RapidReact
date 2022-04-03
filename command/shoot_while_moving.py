from robotpy_toolkit_7407 import Command
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import rad, s, m

from command.drivetrain import curve
from robot_systems import Robot
from subsystem import Drivetrain, Shooter


class ShootWhileMoving(Command):
    def __init__(self, drivetrain: Drivetrain, shooter: Shooter) -> None:
        super().__init__()
        super().addRequirements(drivetrain, shooter)
        self.drivetrain = drivetrain
        self.shooter = shooter

    def initialize(self) -> None:
        Robot.limelight.ref_on()

    def execute(self) -> None:
        dx, dy = self.drivetrain.axis_dx.value, self.drivetrain.axis_dy.value

        robot_vel = self.drivetrain.chassis_speeds.vx, self.drivetrain.chassis_speeds.vy

        hub_dist = Robot.limelight.calculate_distance()

        if hub_dist is None:
            hub_dist = 2  # TODO USE ODOMETRY
            logger.info("NO HUB DISTANCE, ESTIMATING 2")

        hub_angle = -Robot.limelight.get_x_offset()

        angle_offset = self.shooter.target_with_motion(hub_dist, hub_angle, robot_vel)

        logger.info(f"offset={angle_offset}")

        omega = 0.07 * angle_offset * rad/s

        self.drivetrain.set(
            (curve(dx) * self.drivetrain.max_vel.asUnit(m/s), curve(-dy) * self.drivetrain.max_vel.asUnit(m/s)),
            omega
        )

    def end(self, interrupted: bool) -> None:
        Robot.limelight.ref_off()
        self.shooter.stop()
        pass

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
