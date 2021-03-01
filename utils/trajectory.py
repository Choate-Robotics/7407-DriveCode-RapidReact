from typing import Type

from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from utils.paths import Path
import constants


def generate_trajectory(path: Type[Path]):
    trajectory_config = TrajectoryConfig(path.max_velocity, path.max_acceleration)
    trajectory_config.setReversed(False)
    kinematics = DifferentialDriveKinematics(constants.track_width_meters)
    trajectory_config.setKinematics(kinematics)
    return TrajectoryGenerator.generateTrajectory(
        path.start_pos, path.waypoints, path.end_pos,
        trajectory_config
    )
