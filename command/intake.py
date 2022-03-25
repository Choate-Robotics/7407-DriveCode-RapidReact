from commands2 import InstantCommand

from robot_systems import Robot


IntakeToggleLeft = lambda: InstantCommand(Robot.intake.toggle_left_intake, Robot.intake)
IntakeToggleRight = lambda: InstantCommand(Robot.intake.toggle_right_intake, Robot.intake)
IntakeDinglebobOn = lambda: InstantCommand(lambda: Robot.intake.dinglebobs_in, Robot.intake)
IntakeDinglebobOff = lambda: InstantCommand(lambda: Robot.intake.dinglebobs_off, Robot.intake)
IntakeToggleReverse = lambda: InstantCommand(Robot.intake.dinglebob_eject_left, Robot.intake)
