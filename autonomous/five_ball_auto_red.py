import dataclasses
import math
from urllib.parse import ParseResult

from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, WaitCommand, ParallelDeadlineGroup
from robotpy_toolkit_7407.command import SubsystemCommand, T
from robotpy_toolkit_7407.utils.units import m, rad, deg, s, ft, inch
from wpimath.geometry import Pose2d, Translation2d
from command import shooter

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.follow_path import FollowPathCustom, RotateInPlace
from command.shooter import TurretAim  # , TurretZero
from autonomous.trajectory import generate_trajectory_without_unum
from command import IndexOn, IndexOff
from command.shooter import ShooterEnableAtDistance
from subsystem import Index
from robot_systems import Robot
from robot import _Robot

# Pose2d x: more negative equals away from driver station
# Pose2d y: more positive equals away from hangar
# Pose2d angle: the angle the movement is towards, 0 is negative y, 90 is negative x

# paths waypoints_____________
initial_robot_pose = Pose2d(-6.75, 4.068152, 0 * deg)
first_path_waypoint = [Translation2d(-6.75, 5.3)]
first_path_end_pose = Pose2d(-7.25, 5.4, 90 * deg)

second_path_start_pose = Pose2d(-7.25, 5.4, -5 * deg)
second_path_waypoint = [Translation2d(-4.9, 5), Translation2d(-4.6, 3.9)]  # 4.0
second_path_end_pose = Pose2d(-3.875, 4.0, -5 * deg)  # 4.2

third_path_start_pose = Pose2d(-3.875, 4.0, -60 * deg)  # 4.2
third_path_waypoint = [Translation2d(-0.6, 3.75)]
third_path_end_pose = Pose2d(-0.1, 4.25, -135 * deg)

# fourth_path_start_pose = Pose2d(-0.1, 4.25, 120 * deg)
# fourth_path_waypoint = [Translation2d(-0.6, 3.5)]
fourth_path_start_pose = Pose2d(-0.6, 3.5, 180 * deg)
fourth_path_end_pose = Pose2d(-3.875, 4.2, 60 * deg)

fifth_path_start_pose = Pose2d(-0.1, 4.25, 120 * deg)
fifth_path_end_pose = Pose2d(-0.6, 3.5, -120 * deg)

# path commands____________
first_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory_without_unum(initial_robot_pose, first_path_waypoint, first_path_end_pose, 10 * m / s,
                                     2 * m / (s * s)),
    0 * deg,  # this is what rotation the robot will be facing in the end
    period=constants.period
)

second_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory_without_unum(second_path_start_pose, second_path_waypoint, second_path_end_pose, 10 * m / s,
                                     2 * m / (s * s)),
    0 * deg,  # counter clockwise is positive
    period=constants.period
)

third_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory_without_unum(third_path_start_pose, third_path_waypoint, third_path_end_pose, 10 * m / s,
                                     2 * m / (s * s)),
    45 * deg,
    period=constants.period
)

fourth_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory_without_unum(fourth_path_start_pose, [], fourth_path_end_pose, 10 * m / s, 3 * m / (s * s)),
    0 * deg,
    period=constants.period
)

fifth_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory_without_unum(fifth_path_start_pose, [], fifth_path_end_pose, 10 * m / s, 3 * m / (s * s)),
    45 * deg,
    period=constants.period
)


# Sensor commands ______________
def zero():
    Robot.drivetrain.n_00.zero()
    Robot.drivetrain.n_01.zero()
    Robot.drivetrain.n_10.zero()
    Robot.drivetrain.n_11.zero()


def resetGyro():
    Robot.drivetrain.gyro.reset_angle()


def gyro_rezero():
    Robot.drivetrain.gyro._gyro.setYaw(math.degrees(Robot.drivetrain.gyro.get_robot_heading()) - 90)


# Intake commands__________
def left_intake_on():
    Robot.intake.left_intake_enable()


def right_intake_on():
    Robot.intake.right_intake_enable()


def right_intake_off():
    Robot.intake.right_intake_disable()


def left_intake_off():
    Robot.intake.left_intake_disable()


# Dinglebob commands__________
def left_dinglebob_in():
    Robot.index.single_dinglebob_in("Left")


def dinglebob_shoot():
    Robot.index.left_dinglebob.set_raw_output(-0.6)
    Robot.index.right_dinglebob.set_raw_output(0.2)


def right_dinglebob_in():
    Robot.index.single_dinglebob_in("Right")


def right_dinglebob_off():
    Robot.index.single_dinglebob_off("Right")


def left_dinglebob_off():
    Robot.index.single_dinglebob_off("Left")


# shooter command____________
def turn_turret_away():
    Robot.shooter.set_turret_angle(1.4)


def turn_turret_towards():
    Robot.shooter.set_turret_angle(3.2)


# the full auto sequence
final_command = SequentialCommandGroup(
    InstantCommand(resetGyro),
    InstantCommand(zero),
    ParallelDeadlineGroup(  # drive back, pick up opponent ball, shoot preloads
        SequentialCommandGroup(
            ParallelCommandGroup(
                first_path,
                InstantCommand(right_intake_on),
            ),
            InstantCommand(left_dinglebob_in),
            WaitCommand(0.6),
            InstantCommand(left_dinglebob_off),
            InstantCommand(right_intake_off)
        ),
        TurretAim(Robot.shooter)
    ),
    ParallelDeadlineGroup(  # eject wrong ball into hangar, sweeps up the next two balls and shoot them
        SequentialCommandGroup(
            ParallelCommandGroup(
                second_path,
                InstantCommand(right_dinglebob_in),
            ),
            InstantCommand(dinglebob_shoot),
            WaitCommand(0.75),
            InstantCommand(left_dinglebob_off),
        ),
        InstantCommand(left_intake_on),
        SequentialCommandGroup(
            ParallelDeadlineGroup(
                WaitCommand(0.6),
                SequentialCommandGroup(
                    WaitCommand(0.1),
                    InstantCommand(turn_turret_away),
                ),
                ShooterEnableAtDistance(Robot.shooter, 5).withTimeout(1.5),
            ),
            ParallelDeadlineGroup(
                WaitCommand(0.2),
                InstantCommand(turn_turret_towards)
            ),
            TurretAim(Robot.shooter)
        )
    ),
    InstantCommand(left_intake_on),
    third_path,
    ParallelCommandGroup(
        SequentialCommandGroup(
            fifth_path,
            fourth_path,
            InstantCommand(left_intake_on),
            InstantCommand(dinglebob_shoot),
            InstantCommand(gyro_rezero),
        ),
        SequentialCommandGroup(
            ParallelDeadlineGroup(
                WaitCommand(2.75),
                InstantCommand(turn_turret_towards)
            ),
            TurretAim(Robot.shooter)
        )
    )
)

routine = AutoRoutine(initial_robot_pose, final_command)
