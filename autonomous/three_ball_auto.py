from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, WaitCommand
from robotpy_toolkit_7407.utils.units import m, rad, deg, s, ft, inch
from wpimath.geometry import Pose2d

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.follow_path import FollowPathCustom, RotateInPlace
from autonomous.trajectory import generate_trajectory_from_pose, TrajectoryEndpoint, generate_trajectory
from command import IndexOn, IndexOff
from command.intake import IntakeDinglebobOn, IntakeDinglebobOff
from command.shooter import ShooterEnableAtDistance
from robot_systems import Robot

initial_gyro_angle = -90 * deg
initial_robot_pose = Pose2d(7.927611, -6.613358, initial_gyro_angle.asNumber(rad))

first_path_end_pose = TrajectoryEndpoint(7.927611 * m - 6 * inch, -8 * m + 1.6 * ft, initial_gyro_angle.asNumber(rad))

second_path_start_pose = first_path_end_pose
second_path_start_pose.angle = 163 * deg
second_path_end_pose = TrajectoryEndpoint((7.927611 - 2) * m, (-8 + 0.6) * m + 2.6 * ft, 163 * deg)

first_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory_from_pose(initial_robot_pose, [], first_path_end_pose, 8 * m/s, 1.5 * m/(s*s)),
    -90 * deg,
    period=constants.period
)

rotate_1 = RotateInPlace(
    Robot.drivetrain,
    -13 * deg,
    0.8,
    period=constants.period
)

second_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory(
        second_path_start_pose,
        [],
        second_path_end_pose,
        10 * m/s,
        4 * m/(s*s)
    ),
    -13 * deg,
    period=constants.period
)

rotate_2 = RotateInPlace(
    Robot.drivetrain,
    -48 * deg,
    0.5,
    period=constants.period
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
        InstantCommand(lambda: Robot.intake.set_right(True), Robot.intake)
    ),
    WaitCommand(0.05),
    ParallelCommandGroup(
        rotate_1,
        InstantCommand(lambda: Robot.intake.set_right(False), Robot.intake)
    ),
    ParallelCommandGroup(
        ShooterEnableAtDistance(Robot.shooter, 2.1),
        WaitCommand(0.6).andThen(IndexOn().alongWith(IntakeDinglebobOn()))
    ).withTimeout(1.5),
    IndexOff(), IntakeDinglebobOff(),
    ParallelCommandGroup(
        second_path,
        InstantCommand(lambda: Robot.intake.set_left(True), Robot.intake)
    ),
    WaitCommand(0.1),
    ParallelCommandGroup(
        rotate_2,
        WaitCommand(0.5).andThen(InstantCommand(lambda: Robot.intake.set_left(False), Robot.intake)),
    ),
    InstantCommand(lambda: Robot.intake.set_left(False), Robot.intake),
    ParallelCommandGroup(
        ShooterEnableAtDistance(Robot.shooter, 2.4),
        WaitCommand(0.6).andThen(IndexOn().alongWith(IntakeDinglebobOn()))
    ).withTimeout(1.5),
    IndexOff(), IntakeDinglebobOff(),
)

routine = AutoRoutine(initial_robot_pose, final_command)
