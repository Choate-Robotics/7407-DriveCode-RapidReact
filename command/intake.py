from commands2 import InstantCommand

from robot_systems import Robot


IntakeToggleLeft = lambda: InstantCommand(Robot.intake.toggle_left, Robot.intake)
IntakeToggleRight = lambda: InstantCommand(Robot.intake.toggle_right, Robot.intake)
IntakeDinglebobOn = lambda: InstantCommand(lambda: Robot.intake.m_top.set_raw_output(0.5), Robot.intake)
IntakeDinglebobOff = lambda: InstantCommand(lambda: Robot.intake.m_top.set_raw_output(0), Robot.intake)
IntakeToggleReverse = lambda: InstantCommand(Robot.intake.toggle_reverse, Robot.intake)
