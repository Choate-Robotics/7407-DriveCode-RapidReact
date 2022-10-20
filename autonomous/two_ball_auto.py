import math

from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand, WaitCommand, ParallelDeadlineGroup
from robotpy_toolkit_7407.utils.units import m, rad, deg, s, inch
from wpimath.geometry import Pose2d

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.follow_path import FollowPathCustom, RotateInPlace
from autonomous.trajectory import TrajectoryEndpoint, generate_trajectory, generate_trajectory_without_unum
from command import IndexOn, IndexOff
from command.shooter import TurretAim
from robot_systems import Robot


initial_robot_pose = Pose2d(4.9, -2.5, -45 * deg)
first_path_end_pose = Pose2d(3.5, -2.5, -45 * deg)

# path commands____________
first_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory_without_unum(initial_robot_pose, [], first_path_end_pose, 10 * m/s, 2 * m/(s*s)),
    0 * deg, # this is what rotation the robot will be facing in the end
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
    Robot.drivetrain.gyro._gyro.setYaw(math.degrees(Robot.drivetrain.gyro.get_robot_heading()) + 50)

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
    Robot.index.right_dinglebob.set_raw_output(0.6)
    Robot.index.left_dinglebob.set_raw_output(-0.2)

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


# final auto sequence
final_command = SequentialCommandGroup(
    InstantCommand(resetGyro),
    InstantCommand(zero),
    InstantCommand(right_intake_on),
    ParallelDeadlineGroup(
        SequentialCommandGroup(
            first_path,
            InstantCommand(dinglebob_shoot),
            WaitCommand(3)
        ),
        TurretAim(Robot.shooter)
    ),
    InstantCommand(gyro_rezero),
    InstantCommand(right_intake_off),
    InstantCommand(left_dinglebob_off),
    InstantCommand(right_dinglebob_off)
)

routine = AutoRoutine(initial_robot_pose, final_command)