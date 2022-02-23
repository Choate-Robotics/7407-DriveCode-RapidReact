from telnetlib import EL

from commands2 import InstantCommand, ParallelCommandGroup, ConditionalCommand, SequentialCommandGroup
from robotpy_toolkit_7407.command import SubsystemCommand, T
from robotpy_toolkit_7407.unum import Unum
from robotpy_toolkit_7407.unum.units import cm
from robotpy_toolkit_7407.utils import logger

import constants
from robot_systems import Robot
from subsystem import Elevator

ElevatorSolenoidExtend = InstantCommand(Robot.elevator.extend_solenoid, Robot.elevator)
ElevatorSolenoidRetract = InstantCommand(Robot.elevator.extend_solenoid, Robot.elevator)
ElevatorSolenoidToggle = InstantCommand(Robot.elevator.solenoid.toggle, Robot.elevator)

ElevatorSetupCommand = SequentialCommandGroup(
    InstantCommand(lambda: Robot.elevator.set_height(constants.elevator_extended_height)),
    ElevatorSolenoidRetract
)


class ElevatorClimbStep1(SubsystemCommand[Elevator]):
    aborted: bool

    def __init__(self, subsystem: T, tolerance: Unum = 0.5 * cm):
        super().__init__(subsystem)
        self.tolerance = tolerance
        self.grabbed = False
        self.setpoint = constants.elevator_pull_down_height
        ElevatorClimbStep1.aborted = False

    def initialize(self) -> None:
        self.grabbed = False
        self.setpoint = constants.elevator_pull_down_height
        ElevatorClimbStep1.aborted = False

    def execute(self) -> None:
        self.subsystem.set_height(self.setpoint)

        if ElevatorClimbStep1.aborted:
            return

        if self.subsystem.bar_on_climb_hooks():
            self.grabbed = True
        if self.subsystem.get_height() <= constants.elevator_min_bar_contact_height:
            if not self.grabbed:
                ElevatorClimbStep1.aborted = True
                self.setpoint = constants.elevator_extended_height

    def isFinished(self) -> bool:
        return abs(self.subsystem.get_height() - self.setpoint) <= self.tolerance


class ElevatorClimbStep2(SubsystemCommand[Elevator]):
    aborted: bool

    def __init__(self, subsystem: T, tolerance: Unum = 0.5 * cm):
        super().__init__(subsystem)
        self.tolerance = tolerance
        self.latched = False
        self.setpoint = constants.elevator_latch_height
        ElevatorClimbStep2.aborted = False

    def initialize(self) -> None:
        self.latched = False
        self.setpoint = constants.elevator_latch_height
        ElevatorClimbStep2.aborted = False
        logger.info("STEP 2")

    def execute(self) -> None:
        self.subsystem.set_height(self.setpoint)

        if ElevatorClimbStep2.aborted:
            return

        if self.subsystem.bar_on_grab_hooks() and not self.latched:
            logger.info("LATCHED")
            self.latched = True

    def isFinished(self) -> bool:
        if abs(self.subsystem.get_height() - self.setpoint) <= self.tolerance:
            if ElevatorClimbStep2.aborted:
                return True
            if not self.latched:
                ElevatorClimbStep2.aborted = True
                self.setpoint = constants.elevator_pull_down_height
                return False
            return True
        return False


class ElevatorClimbStep3(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T, tolerance: Unum = 0.5 * cm):
        super().__init__(subsystem)
        self.tolerance = tolerance
        self.fired = False

    def initialize(self) -> None:
        logger.info("STEP 3")
        self.fired = False

    def execute(self) -> None:
        self.subsystem.set_height(constants.elevator_extended_height)
        if not self.fired and self.subsystem.get_height() >= constants.elevator_fire_height:
            self.fired = True
            self.subsystem.extend_solenoid()

    def isFinished(self) -> bool:
        return abs(self.subsystem.get_height() - constants.elevator_extended_height) <= self.tolerance


def abort_fn():
    logger.info("ABORTED")


ElevatorClimbCommand = SequentialCommandGroup(
    ElevatorClimbStep1(Robot.elevator),
    ConditionalCommand(ElevatorClimbStep2(Robot.elevator), InstantCommand(abort_fn), lambda: not ElevatorClimbStep1.aborted),
    ConditionalCommand(
        ElevatorClimbStep3(Robot.elevator),
        InstantCommand(abort_fn),
        lambda: not ElevatorClimbStep1.aborted and not ElevatorClimbStep2.aborted
    ),
)
