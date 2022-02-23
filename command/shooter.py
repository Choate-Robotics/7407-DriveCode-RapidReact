from commands2 import InstantCommand
from robot_systems import Robot


ShooterEnable = InstantCommand(lambda: Robot.shooter.target(3.33*2.15), Robot.shooter)
ShooterStop = InstantCommand(lambda: Robot.shooter.stop(), Robot.shooter)
