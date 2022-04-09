import math

from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, WaitCommand
from robotpy_toolkit_7407.utils.units import m, rad, deg, s, ft, inch
from wpimath.geometry import Pose2d, Translation2d

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.follow_path import FollowPathCustom, RotateInPlace
from autonomous.trajectory import generate_trajectory_from_pose, TrajectoryEndpoint, generate_trajectory
from command import ShooterEnable, IndexOn, IndexOff
from command.intake import IntakeDinglebobOn, IntakeDinglebobOff
from command.shooter import ShooterEnableAtDistance
from robot_systems import Robot

initial_gyro_angle = -0.620351 * rad
initial_robot_pose = Pose2d(6.375163, -2.861949, initial_gyro_angle.asNumber(rad))

first_path_angle = 135 * deg
first_path_start_pose = TrajectoryEndpoint(initial_robot_pose.X() * m, initial_robot_pose.Y() * m, first_path_angle)
first_path_end_pose = TrajectoryEndpoint(6.375163 * m - 49 * inch, -2.861949 * m + 21 * inch, first_path_angle)


first_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory(first_path_start_pose, [], first_path_end_pose, 1 * m/s, 2 * m/(s*s)),
    -45 * deg,
    period=constants.period
)


first_turn = RotateInPlace(
    Robot.drivetrain,
    -115 * deg,
    1,
    constants.period
)


def zero():
    Robot.drivetrain.n_00.zero()
    Robot.drivetrain.n_01.zero()
    Robot.drivetrain.n_10.zero()
    Robot.drivetrain.n_11.zero()


final_command = SequentialCommandGroup(
    InstantCommand(zero),
    WaitCommand(0.3),
    ParallelCommandGroup(
        first_path,
        InstantCommand(lambda: Robot.intake.toggle_left_intake(), Robot.intake)
    ),
    ParallelCommandGroup(
        first_turn,
        WaitCommand(0.75).andThen(InstantCommand(lambda: Robot.intake.toggle_left_intake(), Robot.intake))
    ),
    ParallelCommandGroup(
        ShooterEnableAtDistance(Robot.shooter, 2.65),
        WaitCommand(0.6).andThen(IndexOn().alongWith(InstantCommand(lambda: Robot.intake.dinglebobs_in(), Robot.intake)))
    ).withTimeout(1.5),
    IndexOff(), InstantCommand(lambda: Robot.intake.dinglebobs_off(), Robot.intake)
)

#routine = AutoRoutine(initial_robot_pose, final_command)

test_command = SequentialCommandGroup(
    ParallelCommandGroup(
        WaitCommand(0.6).andThen(IndexOn().alongWith(InstantCommand(lambda: Robot.intake.dinglebobs_in(), Robot.intake)))
    ).withTimeout(1.5)
)

routine = AutoRoutine(initial_robot_pose, final_command)