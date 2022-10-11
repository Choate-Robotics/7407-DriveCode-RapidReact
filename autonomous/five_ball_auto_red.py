import dataclasses
import math
#from commands1 import WaitUntilCommand

from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, WaitCommand
from robotpy_toolkit_7407.utils.units import m, rad, deg, s, ft, inch
from wpimath.geometry import Pose2d

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.follow_path import FollowPathCustom, RotateInPlace
from autonomous.trajectory import generate_trajectory_from_pose, TrajectoryEndpoint, generate_trajectory
from command import IndexOn, IndexOff
from command.shooter import ShooterEnableAtDistance
from robot_systems import Robot

initial_gyro_angle = 90 * deg

old_initial_robot_pose = Pose2d(7.927611, -6.613358, initial_gyro_angle.asNumber(rad))
initial_robot_pose = Pose2d(8, 1.7714, initial_gyro_angle.asNumber(rad))

dx = (initial_robot_pose.X() - old_initial_robot_pose.X()) * m
dy = (initial_robot_pose.Y() - old_initial_robot_pose.Y()) * m

first_path_end_pose = TrajectoryEndpoint(7.927611 * m - 6 * inch + dx, -8 * m + 1.6 * ft + dy, initial_gyro_angle.asNumber(rad))

second_path_start_pose = first_path_end_pose
second_path_start_pose.angle = 90 * deg
second_path_end_pose = TrajectoryEndpoint((7.927611 - 2) * m - 7 * inch + dx, (-8 + 0.6) * m + 2.6 * ft + 6 * inch + dy, 90 * deg)

third_path_start_pose = second_path_end_pose
third_path_start_pose.angle = 90 * deg
third_path_end_pose = TrajectoryEndpoint((7.927611 - 2) * m - 175.5 * inch + dx, (-8 + 0.6) * m + 2.6 * ft + 7 * inch + dy, 90 * deg)

fourth_path_start_pose = dataclasses.replace(third_path_end_pose)
fourth_path_start_pose.angle = 0 * deg
fourth_path_end_pose = dataclasses.replace(second_path_end_pose)
fourth_path_end_pose.angle = 0 * deg

first_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory_from_pose(initial_robot_pose, [], first_path_end_pose, 8 * m/s, 1.5 * m/(s*s)),
    90 * deg, # -90 
    period=constants.period
)

rotate_1 = RotateInPlace(
    Robot.drivetrain,
    90 * deg, #-12
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
    90 * deg, # -13
    period=constants.period
)

rotate_2 = RotateInPlace(
    Robot.drivetrain,
    -51 * deg, # -51
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
    90 * deg, # 35
    period=constants.period
)

rotate_3 = RotateInPlace(
    Robot.drivetrain,
    -72.5 * deg,
    1,
    period=constants.period
)

fourth_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory(
        fourth_path_start_pose,
        [],
        fourth_path_end_pose,
        9 * m/s,
        5 * m/(s*s)
    ),
    -49 * deg,
    period=constants.period
)


def right_intake_on():
    Robot.intake.right_intake_enable()
    
def right_dinglebob_in():
    Robot.index.single_dinglebob_in("Right")

def right_dinglebob_out():
    Robot.index.single_dinglebob_out("Right")

def right_intake_off():
    Robot.intake.right_intake_disable()

def right_dinglebob_off():
    Robot.index.single_dinglebob_off("Right")

def left_intake_on():
    Robot.intake.left_intake_enable()

def left_dinglebob_in():
    Robot.index.single_dinglebob_in("Left")

def left_dinglebob_out():
    Robot.index.single_dinglebob_out("Left")

def left_intake_off():
    Robot.intake.left_intake_disable()

def left_dinglebob_off():
    Robot.index.single_dinglebob_off("Left")

def get_right_photo_electric() -> bool:
    if Robot.index.left_limit.get_value():
        return True
    else:
        return False

def get_right_photo_electric() -> bool:
    if Robot.index.right_limit.get_value():
        return True
    else:
        return False

def get_stage_photo_electric() -> bool:
    if Robot.index.photo_electric.get_value():
        return True
    else:
        return False

def dinglebob_travel_left(): #Moving ball from Right -> Left
    Robot.index.single_dinglebob_out("Left")
    Robot.index.single_dinglebob_in("Right")

def dinglebob_travel_right(): #Moving ball from Left -> Right
    Robot.index.single_dinglebob_out("Right")
    Robot.index.single_dinglebob_in("Left")

def dinglebobs_off(): 
    Robot.index.single_dinglebob_off("Left")
    Robot.index.single_dinglebob_off("Right")

def zero():
    Robot.drivetrain.n_00.zero()
    Robot.drivetrain.n_01.zero()
    Robot.drivetrain.n_10.zero()
    Robot.drivetrain.n_11.zero()


def rezero():
    Robot.drivetrain.gyro._gyro.setYaw(math.degrees(Robot.drivetrain.gyro.get_robot_heading()) + 90)


final_command = SequentialCommandGroup(
    InstantCommand(zero),
    WaitCommand(0.3),
    ParallelCommandGroup(
        #maybe turret zero here
        first_path,
        InstantCommand(right_intake_on, Robot.intake),
        InstantCommand(right_dinglebob_in, Robot.index) # could use timeout or wait until interrupt from a specific photo electric sensor... or why not both? ;)
    ),
    WaitCommand(0.05),
    ParallelCommandGroup(
        SequentialCommandGroup(
            rotate_1
            #IndexOn().alongWith(InstantCommand(right_dinglebob_in, Robot.index))
        ),
        ShooterEnableAtDistance(Robot.shooter, 2.4) # Was 2.7
    ).withTimeout(1.5),
    InstantCommand(right_intake_off, Robot.intake),
    InstantCommand(right_dinglebob_off, Robot.index),
    ParallelCommandGroup(
        second_path,
        InstantCommand(left_intake_on, Robot.intake)
    ),
    WaitCommand(0.1),
    ParallelCommandGroup(
        SequentialCommandGroup(
            ParallelCommandGroup(
                rotate_2,
                WaitCommand(0.5).andThen(InstantCommand(left_intake_off, Robot.intake)),
            ),
            IndexOn().alongWith(InstantCommand(lambda: left_dinglebob_in, Robot.intake))
        ),
        ShooterEnableAtDistance(Robot.shooter, 2.8) # CHANGED FROM 3.1 - SID JUN 3 2022
    ).withTimeout(1.5),
    InstantCommand(lambda: left_dinglebob_off, Robot.intake),
    ParallelCommandGroup(
        third_path,
        InstantCommand(left_intake_on, Robot.intake)
    ),
    InstantCommand(Robot.drivetrain.stop, Robot.drivetrain),
    WaitCommand(1.5),
    # WaitCommand(1),
    # ParallelCommandGroup(
    #     SequentialCommandGroup(
    #         ParallelCommandGroup(
    #             WaitCommand(0.5).andThen(InstantCommand(left_intake_off, Robot.intake)),
    #             rotate_3,
    #         ),
    #         IndexOn().alongWith(InstantCommand(lambda: Robot.index.dinglebobs_in(), Robot.intake)),
    #     ),
    #     ShooterEnableAtDistance(Robot.shooter, (23.5 * ft - 8 * inch).asNumber(m))
    # ).withTimeout(2),
    # IndexOff(), InstantCommand(lambda: Robot.index.dinglebobs_off(), Robot.intake),
    ParallelCommandGroup(
        SequentialCommandGroup(
            fourth_path,
            InstantCommand(rezero),
            InstantCommand(lambda: left_dinglebob_in, Robot.intake)
        ),
        ShooterEnableAtDistance(Robot.shooter, 2.7), # Was 3
        WaitCommand(1).andThen(InstantCommand(left_intake_off)),
    )
)

routine = AutoRoutine(initial_robot_pose, final_command)
