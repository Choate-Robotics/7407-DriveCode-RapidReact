from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, WaitCommand
from robotpy_toolkit_7407.utils.units import m, rad, deg, s, ft, inch
from wpimath.geometry import Pose2d, Translation2d

import constants
from autonomous.follow_path import FollowPathCustom, RotateInPlace
from autonomous.trajectory import generate_trajectory_from_pose, TrajectoryEndpoint, generate_trajectory
from command import ShooterEnable, IndexOn, IndexOff
from command.intake import IntakeThingyOn, IntakeThingyOff
from robot_systems import Robot

waypoints = [
             [(7.148493454261461, 6.842319619162739, 0.26179938779914913 , 0), (7.507914912855612, 7.441355383486324), (7.5212268187294695, 8.0, 1.5707963267948963, 0)],
             [(7.5212268187294695, 8.0, 1.5707963267948963, 0), (6.882255336784311, 7.627722065720329), (5.2848266319214146, 6.7890719956673085, 0.7853981633974483, 0)],
             [(5.2848266319214146, 6.7890719956673085, 0.7853981633974478, 0), (5.031900420318123, 6.442962442947015), (4.778974208714831, 6.123476701974435, 1.0471975511965976, 0)]
             ]

initial_gyro_angle = -90 * deg
initial_robot_pose = Pose2d(7.927611, -6.613358, initial_gyro_angle.asNumber(rad))

first_path_end_pose = TrajectoryEndpoint(7.927611 * m, -8 * m + 1.6 * ft, initial_gyro_angle.asNumber(rad))

second_path_start_pose = first_path_end_pose
second_path_start_pose.angle = 163 * deg
second_path_end_pose = TrajectoryEndpoint((7.927611 - 2) * m, (-8 + 0.6) * m + 2.6 * ft, 163 * deg)

first_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory_from_pose(initial_robot_pose, [], first_path_end_pose, 4 * m/s, 1 * m/(s*s)),
    -90 * deg,
    period=constants.period
)

rotate_1 = RotateInPlace(
    Robot.drivetrain,
    -8 * deg,
    1,
    period=constants.period
)

second_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory(
        second_path_start_pose,
        [],
        second_path_end_pose,
        4 * m/s,
        1 * m/(s*s)
    ),
    -13 * deg,
    period=constants.period
)

rotate_2 = RotateInPlace(
    Robot.drivetrain,
    -48 * deg,
    1,
    period=constants.period
)


final_command = SequentialCommandGroup(
    ParallelCommandGroup(
        first_path,
        InstantCommand(lambda: Robot.intake.set_right(True), Robot.intake)
    ),
    ParallelCommandGroup(
        rotate_1,
        InstantCommand(lambda: Robot.intake.set_right(False), Robot.intake)
    ),
    ParallelCommandGroup(
        ShooterEnable(Robot.shooter, 2),
        WaitCommand(1).andThen(IndexOn().alongWith(IntakeThingyOn()))
    ).withTimeout(3),
    IndexOff(), IntakeThingyOff(),
    ParallelCommandGroup(
        second_path,
        InstantCommand(lambda: Robot.intake.set_left(True), Robot.intake)
    ),
    InstantCommand(lambda: Robot.intake.set_left(False), Robot.intake),
    rotate_2,
    ParallelCommandGroup(
        ShooterEnable(Robot.shooter, 2.4),
        WaitCommand(1).andThen(IndexOn().alongWith(IntakeThingyOn()))
    ).withTimeout(3),
    IndexOff(), IntakeThingyOff()
)
