from typing import List, Tuple, Type

from wpimath.geometry import Pose2d, Translation2d
from utils.math import ft_to_m
import math


class Path:
    start_pos: Pose2d
    waypoints: List[Pose2d]
    end_pos: Pose2d
    robot_start_pos: Tuple[float, float]
    cones: List[str]
    special_cones: List[str]
    start_zone: Tuple[str, str]
    end_zone: Tuple[str, str]
    max_velocity: float
    max_acceleration: float


class BarrelRacing(Path):
    start_pos = Pose2d(0, 0, 0)
    waypoints = [
        Translation2d(ft_to_m(11.5), ft_to_m(0)),
        Translation2d(ft_to_m(12.5), ft_to_m(4)),
        Translation2d(ft_to_m(8.5), ft_to_m(4.5)),
        Translation2d(ft_to_m(9), ft_to_m(1)),
        Translation2d(ft_to_m(18), ft_to_m(0)),
        Translation2d(ft_to_m(21), ft_to_m(-3)),
        Translation2d(ft_to_m(18), ft_to_m(-5)),
        Translation2d(ft_to_m(16), ft_to_m(-2)),
        Translation2d(ft_to_m(22), ft_to_m(4)),
        Translation2d(ft_to_m(27), ft_to_m(3)),
        Translation2d(ft_to_m(22), ft_to_m(0)),
    ]
    end_pos = Pose2d(ft_to_m(0), ft_to_m(0.5), math.pi)
    robot_start_pos = (1.5, 7.5)
    cones = ["D2", "B2", "D5", "B8", "D10"]
    special_cones = []
    start_zone = ("D0", "B2")
    end_zone = ("D0", "B2")
    max_velocity = 2
    max_acceleration = 1


class Slalom(Path):
    start_pos = Pose2d(0, 0, 0)
    waypoints = [
        Translation2d(ft_to_m(4.5), ft_to_m(-0.5)),
        Translation2d(ft_to_m(7.5), ft_to_m(-5)),
        Translation2d(ft_to_m(19), ft_to_m(-5)),
        Translation2d(ft_to_m(22.5), ft_to_m(-1)),
        Translation2d(ft_to_m(26.5), ft_to_m(-3)),
        Translation2d(ft_to_m(23), ft_to_m(-5.5)),
        Translation2d(ft_to_m(20), ft_to_m(-0.75)),
        Translation2d(ft_to_m(9), ft_to_m(-0.25)),
        Translation2d(ft_to_m(5), ft_to_m(-4.5)),
    ]
    end_pos = Pose2d(ft_to_m(0), ft_to_m(-5), math.pi)
    robot_start_pos = (2, 2.5)
    cones = ["B1", "B2", "D1", "D2", "D4", "D5", "D6", "D7", "D8", "D10"]
    special_cones = []
    start_zone = ("F0", "D2")
    end_zone = ("D0", "B2")
    max_velocity = 2.375
    max_acceleration = 1.5


class Bounce(Path):
    start_pos = Pose2d(0, 0, 0)
    waypoints = [
        Translation2d(ft_to_m(6), ft_to_m(0)),
        Translation2d(ft_to_m(6), ft_to_m(-3)),
        Translation2d(ft_to_m(6), ft_to_m(-5)),
        Translation2d(ft_to_m(6), ft_to_m(-2)),
    ]
    end_pos = Pose2d(ft_to_m(27), ft_to_m(0), 0)
    robot_start_pos = (1.5, 7.5)
    cones = ["B1", "B2", "B4", "B5", "B7", "B8", "B10", "B11", "D1", "D2", "D3", "D5", "D7", "D8", "D10", "D11", "E3"]
    special_cones = ["A3", "A6", "A9"]
    start_zone = ("D0", "B2")
    end_zone = ("D10", "B12")
    max_velocity = 2
    max_acceleration = 1


CURRENT_PATH: Type[Path] = Slalom
