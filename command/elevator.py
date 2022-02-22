from telnetlib import EL

from commands2 import InstantCommand, ParallelCommandGroup, ConditionalCommand, SequentialCommandGroup
from robotpy_toolkit_7407.command import SubsystemCommand, T
from robotpy_toolkit_7407.unum import Unum
from robotpy_toolkit_7407.unum.units import cm

import constants
from robot_systems import Robot
from subsystem import Elevator

ElevatorSolenoidExtend = InstantCommand(Robot.elevator.extend_solenoid, Robot.elevator)
ElevatorSolenoidRetract = InstantCommand(Robot.elevator.extend_solenoid, Robot.elevator)
ElevatorSolenoidToggle = InstantCommand(Robot.elevator.solenoid.toggle(), Robot.elevator)

ElevatorSetupCommand = ParallelCommandGroup(
    InstantCommand(lambda: Robot.elevator.set_height(constants.elevator_extended_height)),
    ElevatorSolenoidRetract
)


class ElevatorClimbStep1(SubsystemCommand[Elevator]):
    aborted: bool

    def __init__(self, subsystem: T, tolerance: Unum = 1 * cm):
        super().__init__(subsystem)
        self.tolerance = tolerance
        ElevatorClimbStep1.aborted = False

    def initialize(self) -> None:
        self.subsystem.set_height(constants.elevator_latch_height)

    def execute(self) -> None:
        # if self.subsystem.get_height() <= constants.elevator_min_bar_contact_height:
        #     if not self.subsystem.bar_on_climb_hooks():
        #         ElevatorClimbStep1.aborted = True
        #         self.subsystem.set_height(constants.elevator_extended_height)
        pass

    def isFinished(self) -> bool:
        return self.subsystem.get_height() < constants.elevator_latch_height + self.tolerance


ElevatorClimbCommand = SequentialCommandGroup(
    ElevatorClimbStep1(Robot.elevator),
    ElevatorSolenoidExtend
)
