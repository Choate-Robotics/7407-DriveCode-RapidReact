from commands2 import SequentialCommandGroup, ParallelCommandGroup, InstantCommand
from robotpy_toolkit_7407.utils.units import m, rad, deg, s
from wpimath.geometry import Pose2d, Translation2d

import constants
from autonomous.auto_routine import AutoRoutine
from commands.drivetrain import FollowPathCustom
from autonomous.trajectory import generate_trajectory_from_pose, TrajectoryEndpoint, generate_trajectory
from robot_systems import Robot

# get the initial gyro angle from logger.info(Robot.drivetrain.odometry.getPose()) ??
initial_gyro_angle = 0.08 * rad 
initial_robot_pose = Pose2d(8.812, -6.436, initial_gyro_angle.asNumber(rad))


# first_path_waypoints = [Translation2d(8.733, -7.7888),
#                         Translation2d(7.253 , -7.801)]
# first_path_end_pose = TrajectoryEndpoint(6.547 * m - 1 * ft, -7.403 * m + 1 * ft,  120 * deg)
# # the angle in the end pose is the final angle of path, the angle in the followPath commands is the final angle of the robot

first_path_waypoints = [Translation2d(8.812 + 0.273, -6.436 - 1.068),
                        Translation2d(9.085 - 2.000, -7.504 - 0.260)]
first_path_end_pose = TrajectoryEndpoint((7.085 - 1.031) * m, (-7.764 + 0.745) * m, -160 * deg)
# if that value doesn't work, try 135 * deg

second_path_start_pose = first_path_end_pose
second_path_end_pose = TrajectoryEndpoint((6.054 - 0.633) * m, (-7.019 + 0.671) * m, 135 * deg)

third_path_start_pose = second_path_end_pose
third_path_waypoints = [Translation2d(5.421 - 2.496, -6.348 + 0.248)]
third_path_end_pose = TrajectoryEndpoint((2.925 - 1.242) * m, (-6.1 - 0.323) * m, -170 * deg)

fourth_path_start_pose = third_path_end_pose
fourth_path_waypoints = [Translation2d(1.683 + 1.788, -6.423 + 0.683)]
fourth_path_end_pose = TrajectoryEndpoint((3.471 + 3.515) * m, (-5.74 - 0.645) * m, -15 * deg)
# if this doesn't work, try 0 * deg


first_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory_from_pose(initial_robot_pose,
                                  first_path_waypoints, 
                                  first_path_end_pose, 
                                  2 * m/s, 
                                  1.5 * m/(s*s)),
    -45 * deg,
    period=constants.period
)

second_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory(second_path_start_pose, 
                        [], 
                        second_path_end_pose, 
                        2 * m/s, 
                        1.5 * m/(s*s)),
    -45 * deg,
    period=constants.period
)

third_path = FollowPathCustom(
    Robot.drivetrain,
    generate_trajectory(third_path_start_pose,
                        third_path_waypoints,
                        third_path_end_pose,
                        2 * m/s, 
                        1.5 * m/(s*s)),
    195 * deg,
    period=constants.period
)

# fourth_path = FollowPathCustom(
    
# )

final_command = SequentialCommandGroup(
    ParallelCommandGroup(
        first_path,
        InstantCommand(lambda: Robot.intake.toggle_left_intake(), Robot.intake),
    )
)


routine = AutoRoutine(initial_robot_pose, final_command)

