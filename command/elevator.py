from telnetlib import EL

from commands2 import InstantCommand, ParallelCommandGroup, ConditionalCommand, SequentialCommandGroup, WaitCommand
from robotpy_toolkit_7407.command import SubsystemCommand, T
from robotpy_toolkit_7407.unum import Unum
from robotpy_toolkit_7407.unum.units import cm
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit
from robotpy_toolkit_7407.utils.units import m, s

import constants
from robot_systems import Robot
from subsystem import Elevator

class ElevatorZero(SubsystemCommand[Elevator]):
    def initialize(self, subsystem):
        self.subsystem = subsystem
        self.subsystem.motors.set_raw_output(-.1)
        self.subsystem.motors.set_raw_output(-.1)
    def execute(self) -> None:
        if self.subsystem.l_elevator[0].get_value() or self.subsystem.l_elevator[1].get_value():
            self.subsystem.zeroed = True
    def isFinished(self) -> bool:
        return self.subsystem.zeroed
    def end(self) -> None:
        self.subsystem.motors.set_sensor_velocity(-.1 * m/s)
        self.subsystem.motors.set_sensor_position(0 * talon_sensor_unit)
    


ElevatorSolenoidExtend = lambda: InstantCommand(Robot.elevator.extend_solenoid, Robot.elevator)
ElevatorSolenoidRetract = lambda: InstantCommand(Robot.elevator.retract_solenoid, Robot.elevator)
ElevatorSolenoidToggle = lambda: InstantCommand(Robot.elevator.solenoid.toggle, Robot.elevator)

ElevatorSetupCommand = lambda: ParallelCommandGroup(
    InstantCommand(lambda: Robot.elevator.set_height(constants.elevator_extended_height)),
    ElevatorSolenoidRetract()
)


class ElevatorClimbStep1(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T, tolerance: Unum = 0.5 * cm):
        super().__init__(subsystem)
        self.tolerance = tolerance
        self.grabbed = False
        self.aborted = False
        self.aborted2 = False
        self.setpoint = constants.elevator_pull_down_height

    def initialize(self) -> None:
        self.grabbed = False
        self.aborted = False
        self.aborted2 = False
        self.setpoint = constants.elevator_pull_down_height

    def execute(self) -> None:
        self.subsystem.set_height(self.setpoint)

        if self.aborted:
            if self.at_setpoint():
                self.aborted = False
                self.aborted2 = True
                self.setpoint = constants.elevator_extended_height
            return

        if self.aborted2:
            if self.at_setpoint():
                self.aborted2 = False
                self.setpoint = constants.elevator_pull_down_height
            return

        if self.subsystem.bar_on_climb_hooks():
            self.grabbed = True
        if self.subsystem.get_height() <= constants.elevator_min_bar_contact_height:
            if not self.grabbed:
                self.aborted = True
                self.aborted2 = False
                self.setpoint = constants.elevator_extended_height

    def isFinished(self) -> bool:
        return self.at_setpoint() and not (self.aborted or self.aborted2)
    
    def at_setpoint(self) -> bool:
        return abs(self.subsystem.get_height() - self.setpoint) <= self.tolerance


class ElevatorClimbStep2(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T, tolerance: Unum = 0.5 * cm):
        super().__init__(subsystem)
        self.tolerance = tolerance
        self.latched = False
        self.aborted = False
        self.setpoint = constants.elevator_latch_height

    def initialize(self) -> None:
        self.latched = False
        self.setpoint = constants.elevator_latch_height
        self.aborted = False
        logger.info("STEP 2")

    def execute(self) -> None:
        self.subsystem.set_height(self.setpoint)

        if self.aborted:
            return

        if self.subsystem.bar_on_grab_hooks() and not self.latched:
            logger.info("LATCHED")
            self.latched = True

    def isFinished(self) -> bool:
        if self.at_setpoint():
            if not self.aborted:
                if not self.latched:
                    self.aborted = True
                    self.setpoint = constants.elevator_pull_down_height
                    return False
                return True
            else:
                self.aborted = False
                self.setpoint = constants.elevator_latch_height
        return False
    
    def at_setpoint(self) -> bool:
        return abs(self.subsystem.get_height() - self.setpoint) <= self.tolerance


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


class ElevatorClimbStep4(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T, tolerance: Unum = 0.5 * cm):
        super().__init__(subsystem)
        self.tolerance = tolerance

    def initialize(self) -> None:
        logger.info("STEP 4")

    def execute(self) -> None:
        self.subsystem.set_height(constants.elevator_latch_height)

    def isFinished(self) -> bool:
        return abs(self.subsystem.get_height() - constants.elevator_latch_height) <= self.tolerance


def abort_fn():
    logger.info("ABORTED")


ElevatorClimbCommand = lambda: SequentialCommandGroup(
    ElevatorClimbStep1(Robot.elevator),
    ElevatorClimbStep2(Robot.elevator),
    ElevatorClimbStep3(Robot.elevator),
    ElevatorSolenoidRetract().andThen(
        SequentialCommandGroup(
            WaitCommand(3),
            ElevatorClimbStep1(Robot.elevator),
            ElevatorClimbStep2(Robot.elevator),
            ElevatorClimbStep3(Robot.elevator),
            ElevatorSolenoidRetract().andThen(
                SequentialCommandGroup(
                    WaitCommand(3),
                    ElevatorClimbStep1(Robot.elevator),
                    ElevatorClimbStep4(Robot.elevator)
                )
            )
        )
    )
)
