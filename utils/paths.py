from typing import List, Tuple, Type

from robotpy_toolkit_7407.utils.math import ft_to_m
from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import Trajectory

import math


class Route:
    start_pos: Pose2d
    waypoints: list[Translation2d]
    end_pos: Pose2d
    flipped: bool

    def __init__(self, start_pos: Pose2d, waypoints: list[Translation2d], end_pos: Pose2d, flipped: bool):
        self.start_pos, self.waypoints, self.end_pos, self.flipped = start_pos, waypoints, end_pos, flipped


class GeneratedRoute:
    route: Route
    trajectory: Trajectory

    def __init__(self, route: Route, trajectory: Trajectory):
        self.route, self.trajectory = route, trajectory


class Path:
    routes: list[Route]
    cones: list[str]
    special_cones: list[str]
    start_zone: tuple[str, str]
    end_zone: tuple[str, str]
    max_velocity: float
    max_acceleration: float


class BarrelRacing(Path):
    routes = [
        Route(
            start_pos=Pose2d(ft_to_m(1.5), ft_to_m(5), 0),
            waypoints=[
                Translation2d(ft_to_m(13), ft_to_m(5)),
                Translation2d(ft_to_m(14), ft_to_m(9)),
                Translation2d(ft_to_m(10), ft_to_m(9.5)),
                Translation2d(ft_to_m(11.5), ft_to_m(6)),
                Translation2d(ft_to_m(19.5), ft_to_m(5)),
                Translation2d(ft_to_m(22.5), ft_to_m(2)),
                Translation2d(ft_to_m(19.5), ft_to_m(0)),
                Translation2d(ft_to_m(17.5), ft_to_m(3)),
                Translation2d(ft_to_m(23.5), ft_to_m(9)),
                Translation2d(ft_to_m(28.5), ft_to_m(8)),
                Translation2d(ft_to_m(22.5), ft_to_m(5)),
            ],
            end_pos=Pose2d(ft_to_m(0 + 1.5), ft_to_m(0.5 + 5), math.pi),
            flipped=False
        )
    ]
    cones = ["D2", "B2", "D5", "B8", "D10"]
    special_cones = []
    start_zone = ("D0", "B2")
    end_zone = ("D0", "B2")
    max_velocity = 2
    max_acceleration = 1


class Slalom(Path):
    routes = [
        Route(
            start_pos=Pose2d(ft_to_m(2), ft_to_m(10.5), 0),
            waypoints=[
                Translation2d(ft_to_m(6.5), ft_to_m(10)),
                Translation2d(ft_to_m(9.5), ft_to_m(5.5)),
                Translation2d(ft_to_m(21), ft_to_m(5.5)),
                Translation2d(ft_to_m(24.5), ft_to_m(9.5)),
                Translation2d(ft_to_m(28.5), ft_to_m(7.5)),
                Translation2d(ft_to_m(25), ft_to_m(5)),
                Translation2d(ft_to_m(22), ft_to_m(9.75)),
                Translation2d(ft_to_m(11), ft_to_m(10.25)),
                Translation2d(ft_to_m(7), ft_to_m(6)),
            ],
            end_pos=Pose2d(ft_to_m(2), ft_to_m(5.5), math.pi),
            flipped=True
        )
    ]
    cones = ["B1", "B2", "D1", "D2", "D4", "D5", "D6", "D7", "D8", "D10"]
    special_cones = []
    start_zone = ("F0", "D2")
    end_zone = ("D0", "B2")
    max_velocity = 2.375
    max_acceleration = 1.5


class Bounce(Path):
    routes = [
        Route(
            start_pos=Pose2d(ft_to_m(2), ft_to_m(5), 0),
            waypoints=[
                Translation2d(ft_to_m(6), ft_to_m(4)),
            ],
            end_pos=Pose2d(ft_to_m(7.5), ft_to_m(0), -math.pi / 2),
            flipped=False
        ),
        Route(
            start_pos=Pose2d(ft_to_m(7.5), ft_to_m(0), math.pi / 2),
            waypoints=[
                Translation2d(ft_to_m(9.5), ft_to_m(6)),
                Translation2d(ft_to_m(10), ft_to_m(10)),
                Translation2d(ft_to_m(15), ft_to_m(10)),
            ],
            end_pos=Pose2d(ft_to_m(15), ft_to_m(0), -math.pi / 2),
            flipped=True
        ),
        Route(
            start_pos=Pose2d(ft_to_m(15), ft_to_m(0), math.pi / 2),
            waypoints=[
                Translation2d(ft_to_m(15), ft_to_m(8)),
                Translation2d(ft_to_m(20), ft_to_m(10)),
                Translation2d(ft_to_m(22.5), ft_to_m(7)),
            ],
            end_pos=Pose2d(ft_to_m(22.5), ft_to_m(0), -math.pi / 2),
            flipped=False
        ),
        Route(
            start_pos=Pose2d(ft_to_m(22.5), ft_to_m(0), math.pi / 2),
            waypoints=[
                Translation2d(ft_to_m(23), ft_to_m(4)),
            ],
            end_pos=Pose2d(ft_to_m(27.5), ft_to_m(5), 0),
            flipped=True
        ),
    ]
    cones = ["B1", "B2", "B4", "B5", "B7", "B8", "B10", "B11", "D1", "D2", "D3", "D5", "D7", "D8", "D10", "D11", "E3"]
    special_cones = ["A3", "A6", "A9"]
    start_zone = ("D0", "B2")
    end_zone = ("D10", "B12")
    max_velocity = 2.25
    max_acceleration = 2


CURRENT_PATH: Type[Path] = Bounce
