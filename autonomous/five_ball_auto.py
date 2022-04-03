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

initial_gyro_angle = -90 * deg
initial_robot_pose = Pose2d(7.927611, -6.613358, initial_gyro_angle.asNumber(rad))

first_path_end_pose = TrajectoryEndpoint(7.927611 * m - 6 * inch, -8 * m + 1.6 * ft, initial_gyro_angle.asNumber(rad))

second_path_start_pose = first_path_end_pose
second_path_start_pose.angle = 163 * deg
second_path_end_pose = TrajectoryEndpoint((7.927611 - 2) * m, (-8 + 0.6) * m + 2.6 * ft, 163 * deg)

third_path_start_pose = second_path_end_pose
third_path_start_pose.angle = 163 * deg
third_path_end_pose = TrajectoryEndpoint((7.927611 - 2) * m - 178.5 * inch, (-8 + 0.6) * m + 2.6 * ft + 4 * inch, -116 * deg)

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

third_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory(
        third_path_start_pose,
        [],
        third_path_end_pose,
        11 * m/s,
        5 * m/(s*s)
    ),
    35 * deg,
    period=constants.period
)

rotate_3 = RotateInPlace(
    Robot.drivetrain,
    -72.5 * deg,
    1,
    period=constants.period
)


final_command = SequentialCommandGroup(
    ParallelCommandGroup(
        first_path,
        InstantCommand(lambda: Robot.intake.toggle_right_intake(), Robot.intake)
    ),
    WaitCommand(0.05),
    ParallelCommandGroup(
        rotate_1,
        InstantCommand(lambda: Robot.intake.toggle_right_intake(), Robot.intake)
    ),
    ParallelCommandGroup(
        ShooterEnableAtDistance(Robot.shooter, 2),
        WaitCommand(0.6).andThen(IndexOn().alongWith(InstantCommand(lambda: Robot.intake.dinglebobs_in(), Robot.intake)))
    ).withTimeout(1.5),
    IndexOff(), InstantCommand(lambda: Robot.intake.dinglebobs_off(), Robot.intake),
    ParallelCommandGroup(
        second_path,
        InstantCommand(lambda: Robot.intake.toggle_left_intake(), Robot.intake)
    ),
    WaitCommand(0.1),
    ParallelCommandGroup(
        rotate_2,
        WaitCommand(0.5).andThen(InstantCommand(lambda: Robot.intake.toggle_left_intake(), Robot.intake)),
    ),
    #InstantCommand(lambda: Robot.intake.set_left(False), Robot.intake),
    ParallelCommandGroup(
        ShooterEnableAtDistance(Robot.shooter, 2.4),
        WaitCommand(0.6).andThen(IndexOn().alongWith(InstantCommand(lambda: Robot.intake.dinglebobs_in(), Robot.intake)))
    ).withTimeout(1.5),
    IndexOff(), InstantCommand(lambda: Robot.intake.dinglebobs_off(), Robot.intake),
    ParallelCommandGroup(
        third_path,
        InstantCommand(lambda: Robot.intake.toggle_left_intake(), Robot.intake)
    ),
    InstantCommand(Robot.drivetrain.stop, Robot.drivetrain),
    WaitCommand(1.3),
    ParallelCommandGroup(
        WaitCommand(0.5).andThen(InstantCommand(lambda: Robot.intake.toggle_left_intake(), Robot.intake)),
        rotate_3,
    ),
    ParallelCommandGroup(
        ShooterEnableAtDistance(Robot.shooter, (21 * ft - 8 * inch).asNumber(m)),
        WaitCommand(0.5).andThen(IndexOn().alongWith(InstantCommand(lambda: Robot.intake.dinglebobs_in(), Robot.intake)))
    ).withTimeout(4),
    IndexOff(), InstantCommand(lambda: Robot.intake.dinglebobs_off(), Robot.intake)
)

routine = AutoRoutine(initial_robot_pose, final_command)
