from dataclasses import dataclass

import commands2
from commands2 import CommandBase
from wpimath.geometry import Pose2d

from robot_systems import Robot


@dataclass
class AutoRoutine:
    initial_robot_pose: Pose2d
    command: CommandBase

    def run(self):
        Robot.drivetrain.odometry.resetPosition(
            self.initial_robot_pose,
            self.initial_robot_pose.rotation()
        )
        Robot.drivetrain.gyro._gyro.setYaw(
            self.initial_robot_pose.rotation().degrees()
        )
        commands2.CommandScheduler.getInstance().schedule(
            self.command
        )
