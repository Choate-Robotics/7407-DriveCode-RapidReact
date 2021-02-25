import wpilib.controller
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import Trajectory, TrajectoryGenerator, TrajectoryConfig
from wpimath.kinematics import DifferentialDriveKinematics
import commands2 as commands
import ctre

import subsystem
import utils.logger as logger
from utils.math import meters_to_sensor_units


class FollowPath(commands.RamseteCommand):
    def __init__(self, drivetrain: subsystem.Drivetrain) -> None:
        trajectory_config = TrajectoryConfig(4, 4)
        trajectory_config.setReversed(False)

        controller = wpilib.controller.RamseteController()

        kinematics = DifferentialDriveKinematics(25.111 * 0.0254)

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
            self._drivetrain.left1.set(ctre.ControlMode.Velocity, left)
            self._drivetrain.right1.set(ctre.ControlMode.Velocity, right)

        super().__init__(
            trajectory,
            lambda: drivetrain.odometry.getPose(),
            controller,
            kinematics,
            output,
            [drivetrain]
        )

        self._drivetrain = drivetrain
