from dataclasses import dataclass

from robotpy_toolkit_7407.unum import Unum
from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory

from robotpy_toolkit_7407.utils.units import m, rad, s


def translation(x: Unum, y: Unum) -> Translation2d:
    return Translation2d(x.asNumber(m), y.asNumber(m))


@dataclass
class TrajectoryEndpoint:
    x: Unum = 0 * m
    y: Unum = 0 * m
    angle: Unum = 0 * rad
    vel: Unum = 0 * m/s

    def as_pose(self):
        return Pose2d(self.x.asNumber(m), self.y.asNumber(m), self.angle.asNumber(rad))


def generate_trajectory(start: TrajectoryEndpoint, waypoints: list[Translation2d], end: TrajectoryEndpoint,
                        max_vel: Unum, max_accel: Unum) -> Trajectory:
    config = TrajectoryConfig(max_vel.asNumber(m/s), max_accel.asNumber(m/(s*s)))
    config.setStartVelocity(start.vel.asNumber(m/s))
    config.setEndVelocity(end.vel.asNumber(m/s))
    return TrajectoryGenerator.generateTrajectory(start.as_pose(), waypoints, end.as_pose(), config)


def generate_trajectory_from_pose(start: Pose2d, waypoints: list[Translation2d], end: TrajectoryEndpoint,
                                  max_vel: Unum, max_accel: Unum) -> Trajectory:
    config = TrajectoryConfig(max_vel.asNumber(m/s), max_accel.asNumber(m/(s*s)))
    config.setStartVelocity(0)
    config.setEndVelocity(end.vel.asNumber(m/s))
    return TrajectoryGenerator.generateTrajectory(start, waypoints, end.as_pose(), config)
