from commands2 import InstantCommand, ParallelCommandGroup, ConditionalCommand, SequentialCommandGroup, WaitCommand
from robotpy_toolkit_7407.command import SubsystemCommand, T
from robotpy_toolkit_7407.utils.units import inch, meters, rev

import constants
from robot_systems import Robot
from subsystem import Elevator


class ElevatorRezero(SubsystemCommand):
    def __init__(self, subsystem: Elevator):
        super().__init__(subsystem)

    def initialize(self):
        pass

    def execute(self):
        if not self.subsystem.mag_sensor.get_value():
            self.subsystem.motors.set_raw_output(-.05)
        else:
            self.subsystem.motors.set_raw_output(0)
            self.subsystem.motors.set_target_position(0)
            self.subsystem.motors.set_sensor_position(0)

    def isFinished(self):
        return self.subsystem.mag_sensor.get_value()


def elevator_down():
    Robot.elevator.set_height(0)
    Robot.drivetrain.max_vel = constants.drivetrain_max_vel


ElevatorDown = lambda: InstantCommand(elevator_down, Robot.elevator)
ElevatorSolenoidExtend = lambda: InstantCommand(Robot.elevator.extend_solenoid, Robot.elevator)
ElevatorSolenoidRetract = lambda: InstantCommand(Robot.elevator.retract_solenoid, Robot.elevator)
ElevatorSolenoidToggle = lambda: InstantCommand(Robot.elevator.solenoid.toggle, Robot.elevator)


def restrict_robot_vel():
    Robot.drivetrain.max_vel = constants.drivetrain_max_climb_vel


def set_initialized():
    Robot.elevator.initialized = True


ElevatorSetupCommand = lambda: ParallelCommandGroup(
    InstantCommand(lambda: restrict_robot_vel()),
    InstantCommand(lambda: Robot.elevator.set_height(constants.elevator_extended_height)),
    InstantCommand(set_initialized),
    ElevatorSolenoidRetract()
)

class WaitUntilTiltRange(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T):
        super().__init__(subsystem)
        self.min_angle = 30
        self.max_angle = 45
        self.prev_angle = 0

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        a = Robot.drivetrain.gyro._gyro.getRoll()
        da = a - self.prev_angle
        if self.min_angle < a < self.max_angle and da > 0:
            finished = True
        elif a > 45 and da < 0:
            finished = True
        else:
            finished = False

        # print(f"ANGLE={a}, da={da}, finished={finished}")
        self.prev_angle = a
        # a = Robot.drivetrain.gyro._gyro.getRoll()
        return finished
        # return False
        # return self.min_angle < a < self.max_angle

    def runsWhenDisabled(self) -> bool:
        return True


def abort_fn():
    pass

class ElevatorDownAllTheWay(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T, tolerance: meters = 0.005):
        super().__init__(subsystem)
        self.tolerance = tolerance
        self.setpoint = constants.elevator_pull_down_height

    def initialize(self) -> None:
        self.setpoint = constants.elevator_pull_down_height

    def execute(self) -> None:
        self.subsystem.set_height(self.setpoint)

    def isFinished(self) -> bool:
        return self.at_setpoint()

    def at_setpoint(self) -> bool:
        return abs(self.subsystem.get_height() - self.setpoint) <= self.tolerance


class ElevatorUpTillTrex(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T, tolerance: meters = 0.005):
        super().__init__(subsystem)
        self.tolerance = tolerance
        self.setpoint = constants.elevator_fire_height

    def initialize(self) -> None:
        self.setpoint = constants.elevator_fire_height

    def execute(self) -> None:
        self.subsystem.set_height(self.setpoint)

    def isFinished(self) -> bool:
        return self.at_setpoint()

    def at_setpoint(self) -> bool:
        return abs(self.subsystem.get_height() - self.setpoint) <= self.tolerance


class ElevatorUpTillBelowExtendedHeight(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T, tolerance: meters = 0.005):
        super().__init__(subsystem)
        self.tolerance = tolerance
        self.setpoint = constants.elevator_below_extended_height

    def initialize(self) -> None:
        self.setpoint = constants.elevator_below_extended_height

    def execute(self) -> None:
        self.subsystem.set_height(self.setpoint)

    def isFinished(self) -> bool:
        return self.at_setpoint()

    def at_setpoint(self) -> bool:
        return abs(self.subsystem.get_height() - self.setpoint) <= self.tolerance


class ElevatorUpTillExtendedHeight(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T, tolerance: meters = 0.005):
        super().__init__(subsystem)
        self.tolerance = tolerance
        self.setpoint = constants.elevator_extended_height

    def initialize(self) -> None:
        self.setpoint = constants.elevator_extended_height

    def execute(self) -> None:
        self.subsystem.set_height(self.setpoint)

    def isFinished(self) -> bool:
        return self.at_setpoint()

    def at_setpoint(self) -> bool:
        return abs(self.subsystem.get_height() - self.setpoint) <= self.tolerance


class ElevatorUpTillMoreBelowExtendedHeight(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T, tolerance: meters = 0.005):
        super().__init__(subsystem)
        self.tolerance = tolerance
        self.setpoint = constants.elevator_more_below_extended_height

    def initialize(self) -> None:
        self.setpoint = constants.elevator_more_below_extended_height

    def execute(self) -> None:
        self.subsystem.set_height(self.setpoint)

    def isFinished(self) -> bool:
        return self.at_setpoint()

    def at_setpoint(self) -> bool:
        return abs(self.subsystem.get_height() - self.setpoint) <= self.tolerance


ElevatorClimbCommand = lambda: SequentialCommandGroup(
    InstantCommand(lambda: Robot.elevator.set_climb_speed(), Robot.elevator),
    ElevatorDownAllTheWay(Robot.elevator),
    ElevatorUpTillTrex(Robot.elevator),
    ElevatorSolenoidExtend(),
    ElevatorUpTillExtendedHeight(Robot.elevator),
    ElevatorSolenoidRetract(),
    WaitCommand(0.5),
    ElevatorDownAllTheWay(Robot.elevator),
    InstantCommand(lambda: Robot.elevator.set_high_climb_speed(), Robot.elevator),
    ElevatorSolenoidExtend(),
    ElevatorUpTillMoreBelowExtendedHeight(Robot.elevator),
    WaitUntilTiltRange(Robot.elevator),
    ElevatorUpTillExtendedHeight(Robot.elevator),
    ElevatorSolenoidRetract(),
    InstantCommand(lambda: Robot.elevator.set_climb_speed(), Robot.elevator),
    WaitCommand(1),
    ElevatorUpTillBelowExtendedHeight(Robot.elevator)
)

"""
E CLimb LOGIC:

1. Extend elevator to the top (Separate commands that can override the climb sequence at any time)

CLIMB COMMAND
1. Retract elevator until set position - POSITION
2. Extend elevator a little bit to catch the trex hooks - POSITION
3. Extend solenoid while extending elevator pass the height of the high bar 
    (so the elevator hooks get on high bar on initial swing) - BOOL/POSITION
4. Retract elevator solenoid - BOOL
5. Wait 0.5 seconds for the swing to calm down - TIME
6. Pull elevator all the way up (trex hooks over the high bar) - POSITION
7. Set the elevator climb speed faster (so we can catch the traversal bar on initial swing)
8. Extend solenoid while moving the elevator half way up (the trex hooks will fall onto the high rung) - BOOL/POSITION
9. Use gyro to check if the elevator is in position to extend over the traversal bar 
    (either just swinging back towards traversal or just swinging past it leaning back) - BOOL
10. Extend elevator pass the height of the traversal bar - POSITION
11. Retract solenoid - BOOL
12. Wait 1 second before pulled the robot off of high bar - TIME/POSITION

"""
