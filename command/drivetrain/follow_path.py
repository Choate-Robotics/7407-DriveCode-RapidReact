import wpilib.controller
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import Trajectory, TrajectoryGenerator, TrajectoryConfig
from wpimath.kinematics import DifferentialDriveKinematics
import commands2 as commands
import ctre

import subsystem
import utils.logger as logger
from utils.math import meters_to_sensor_units

import constants


class FollowPath(commands.RamseteCommand):
    def __init__(self, drivetrain: subsystem.Drivetrain) -> None:
        trajectory_config = TrajectoryConfig(4, 4)
        trajectory_config.setReversed(False)

        controller = wpilib.controller.RamseteController()

        kinematics = DifferentialDriveKinematics(constants.track_width_meters)

        waypoints = (
            Pose2d(0, 0, 0),  # START
            [
                Translation2d(10, 10)
            ],
            Pose2d(20, 20, 0),  # END
        )

        trajectory = TrajectoryGenerator.generateTrajectory(
            waypoints[0], waypoints[1], waypoints[2],
            trajectory_config
        )

        def output(left: float, right: float):
            left = meters_to_sensor_units(left, True) / 10  # m/s to sensor/100ms
            right = meters_to_sensor_units(right, True) / 10
            self._drivetrain.set_motor_velocity(left, -right)

        def get_pose():
            pose = drivetrain.get_pose()
            logger.info(f"{pose.X()}, {pose.Y()}")
            return pose

        super().__init__(
            trajectory,
            get_pose,
            controller,
            kinematics,
            output,
            [drivetrain]
        )

        drivetrain.reset_pose()

        self._drivetrain = drivetrain
