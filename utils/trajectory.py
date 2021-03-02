from typing import Type, List

from wpimath.geometry import Pose2d
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from utils.paths import Path, Route, GeneratedRoute
import constants


def generate_trajectory(path: Type[Path]) -> List[GeneratedRoute]:
    return list(generate_route(path, r) for r in path.routes)


def generate_route(path: Type[Path], route: Route) -> GeneratedRoute:
    trajectory_config = TrajectoryConfig(path.max_velocity, path.max_acceleration)
    trajectory_config.setReversed(False)
    kinematics = DifferentialDriveKinematics(constants.track_width_meters)
    trajectory_config.setKinematics(kinematics)
    trajectory = TrajectoryGenerator.generateTrajectory(
        route.start_pos, route.waypoints, route.end_pos,
        trajectory_config
    )
    return GeneratedRoute(route, trajectory)
