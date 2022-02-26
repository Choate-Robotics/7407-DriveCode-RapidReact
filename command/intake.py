from commands2 import InstantCommand

from robot_systems import Robot


IntakeToggle = InstantCommand(lambda: Robot.intake.toggle(), Robot.intake)
