from telnetlib import EL

from commands2 import InstantCommand
from robotpy_toolkit_7407.command import SubsystemCommand

from robot_systems import Robot
from subsystem import Elevator


ElevatorUp = InstantCommand(Robot.elevator.up(), Robot.elevator)
ElevatorDown = InstantCommand(Robot.elevator.down(), Robot.elevator)
ElevatorStop = InstantCommand(Robot.elevator.stop(), Robot.elevator)
ElevatorSolenoidToggle = InstantCommand(Robot.elevator.toggle_solenoid(), Robot.elevator)
