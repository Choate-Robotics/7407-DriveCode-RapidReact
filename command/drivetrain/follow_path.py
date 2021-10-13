import math
from typing import Type

import wpilib.controller
from wpimath.kinematics import DifferentialDriveKinematics
import commands2 as commands

import subsystem
from robot_systems import robot
from utils.math import meters_to_sensor_units

import constants
from utils.network import Network
from utils.paths import Path, GeneratedRoute
from utils.trajectory import generate_trajectory


def get_command(path: Type[Path]):
    command = None
    trajectories = generate_trajectory(path)

    for trajectory in trajectories:
        current_command = FollowPath(robot.drivetrain, trajectory, trajectory.route.flipped)

        if command is None:
            command = current_command
        else:
            command = command.andThen(current_command)

    return command


class FollowPath(commands.RamseteCommand):
    def __init__(self, generated_trajectory: GeneratedRoute, flipped: bool = False) -> None:
        controller = wpilib.controller.RamseteController()
        kinematics = DifferentialDriveKinematics(constants.track_width_meters)

        trajectory = generated_trajectory.trajectory.relativeTo(generated_trajectory.route.start_pos)

        def output(left: float, right: float):
            left = meters_to_sensor_units(left, True) / 10  # m/s to sensor/100ms
            right = meters_to_sensor_units(right, True) / 10
            Network.test_table.putNumber("vel_left", left)
            Network.test_table.putNumber("vel_right", right)
            self._drivetrain.set_motor_velocity(-left, right)

        def get_pose():
            pose = robot.drivetrain.get_pose()
            return pose

        super().__init__(
            trajectory,
            get_pose,
            controller,
            kinematics,
            output,
            [robot.drivetrain]
        )

        self._flipped = flipped

    def initialize(self) -> None:
        robot.drivetrain.reset_pose(self._flipped)
        super().initialize()


