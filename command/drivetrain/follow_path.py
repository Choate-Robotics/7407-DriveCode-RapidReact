import math

import wpilib.controller
from wpimath.kinematics import DifferentialDriveKinematics
import commands2 as commands

import subsystem
from utils.math import meters_to_sensor_units

import constants
from utils.network import Network
from utils.paths import CURRENT_PATH as PATH
from utils.trajectory import generate_trajectory


class FollowPath(commands.RamseteCommand):
    def __init__(self, drivetrain: subsystem.Drivetrain) -> None:
        controller = wpilib.controller.RamseteController()
        kinematics = DifferentialDriveKinematics(constants.track_width_meters)

        trajectory = generate_trajectory(PATH)

        def output(left: float, right: float):
            left = meters_to_sensor_units(left, True) / 10  # m/s to sensor/100ms
            right = meters_to_sensor_units(right, True) / 10
            Network.test_table.putNumber("vel_left", left)
            Network.test_table.putNumber("vel_right", right)
            self._drivetrain.set_motor_velocity(-left, right)

        def get_pose():
            pose = drivetrain.get_pose()
            return pose

        super().__init__(
            trajectory,
            get_pose,
            controller,
            kinematics,
            output,
            [drivetrain]
        )

        self._drivetrain = drivetrain

        drivetrain.reset_pose()
