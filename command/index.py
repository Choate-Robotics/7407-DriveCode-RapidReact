from commands2 import InstantCommand

from robot_systems import Robot


IndexOn = InstantCommand(Robot.index.set(.5), Robot.index)
IndexOff = InstantCommand(Robot.index.set(0), Robot.index)
