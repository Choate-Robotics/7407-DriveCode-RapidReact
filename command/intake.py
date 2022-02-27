from commands2 import InstantCommand

from robot_systems import Robot


IntakeToggleLeft = lambda: InstantCommand(lambda: Robot.intake.toggle_left(), Robot.intake)
IntakeToggleRight = lambda: InstantCommand(lambda: Robot.intake.toggle_right(), Robot.intake)
