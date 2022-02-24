from commands2 import InstantCommand

from robot_systems import Robot


IntakeOn = InstantCommand(lambda: Robot.intake.set(.5, 0.7), Robot.intake)
IntakeOff = InstantCommand(lambda: Robot.intake.set(0, 0), Robot.intake)
IntakeLeftSolenoidToggle = InstantCommand(Robot.intake.toggle_left_intake, Robot.intake)
IntakeRightSolenoidToggle = InstantCommand(Robot.intake.toggle_right_intake, Robot.intake)
