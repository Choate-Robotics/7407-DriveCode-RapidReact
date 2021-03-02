import math

from wpimath.geometry import Pose2d, Rotation2d

import utils.logger as logger
from utils.math import sensor_units_to_meters, clamp, ft_to_m
from utils.network import Network
import subsystem.drivetrain


class SimDrivetrain(subsystem.drivetrain.Drivetrain):
    LEFT_VEL_METERS_PER_SECOND: float = 0
    RIGHT_VEL_METERS_PER_SECOND: float = 0
    POSE: Pose2d = Pose2d(ft_to_m(2.5), ft_to_m(7.5), 0)

    def __init__(self) -> None:
        super().__init__()
        logger.info("initializing drivetrain", "[drivetrain]")

        self.origin_x = 0
        self.origin_y = 0
        self.origin_theta = Rotation2d(0)

        self.flipped = False

        self.reset_pose()

        logger.info("initialization complete", "[drivetrain]")

    def set_motor_percent_output(self, left: float, right: float):
        self.set_motor_velocity(left * 20000, right * 20000)

    def set_motor_velocity(self, left: float, right: float):
        left = clamp(left, -20000, 20000)
        right = clamp(right, -20000, 20000)
        if self.flipped:
            left, right = right, left
        SimDrivetrain.LEFT_VEL_METERS_PER_SECOND = -sensor_units_to_meters(left, True) * 10
        SimDrivetrain.RIGHT_VEL_METERS_PER_SECOND = sensor_units_to_meters(right, True) * 10

    def reset_pose(self, flipped: bool = False):
        logger.info(f"resetting pose{' (flipped)' if flipped else ''}....")
        self.flipped = flipped
        self.origin_x = SimDrivetrain.POSE.X()
        self.origin_y = SimDrivetrain.POSE.Y()
        self.origin_theta = SimDrivetrain.POSE.rotation()

    def get_pose(self):
        relative_pose = SimDrivetrain.POSE.relativeTo(Pose2d(self.origin_x, self.origin_y, self.origin_theta))
        if self.flipped:
            relative_pose = Pose2d(-relative_pose.X(), -relative_pose.Y(), relative_pose.rotation())
        return relative_pose

    def periodic(self) -> None:
        pose = self.get_pose()
        Network.test_table.putNumber("pose_x", pose.X())
        Network.test_table.putNumber("pose_y", pose.Y())
        Network.test_table.putNumber("pose_degrees", pose.rotation().degrees())
        Network.test_table.putBoolean("flipped", self.flipped)

    def update_odometry(self):
        pass
