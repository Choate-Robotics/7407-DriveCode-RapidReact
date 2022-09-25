from networktables import NetworkTables
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit
from robotpy_toolkit_7407.utils.units import deg, rad, s, m

from robot_systems import Robot
from subsystem import Shooter

import math


class ShooterEnable(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        d = Robot.odometry.hub_dist
        if d is None:
            d = 2

        self.subsystem.target_stationary(d)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()


class ShooterEnableAtDistance(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter, distance: float):
        super().__init__(subsystem)
        self.distance = distance

    def initialize(self) -> None:
        self.subsystem.target_stationary(self.distance)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()


class ShooterZero(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.m_angle.set_raw_output(-0.06)

    def execute(self):
        if self.subsystem.left_limit.get_value():
            self.subsystem.zeroed = True

    def isFinished(self) -> bool:
        return self.subsystem.zeroed

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()
        self.subsystem.m_angle.set_sensor_position(0 * talon_sensor_unit)


class TurretAim(SubsystemCommand[Shooter]):
    def __init__(self, subsystem, ready_counts=2):
        super().__init__(subsystem)
        self.old_offset = .01
        self.old_offset_ratio = 0
        self.ready_counts = ready_counts
        self.c_count = 0
        self.power = 0
        self.max_power = .30
        self.min_power = -.30
        self.min_movement_power = .3

        self.new_run = True
        self.initial_offset = 1

        self.multiplier = .9

    def initialize(self) -> None:
        # self.old_offset = Robot.limelight.table.getNumber('tx', None)
        self.power = 0

    def execute(self) -> None:

        current_offset = Robot.limelight.table.getNumber('tx', None)

        if abs(current_offset) > 3:
            if self.new_run:
                print("NEW RUN, SETTING INITIAL OFFSET TO CURRENT OFFSET")
                self.initial_offset = int(current_offset)
                self.power = self.min_movement_power
            self.new_run = False

            if current_offset > 0:
                sign = 1
            elif current_offset < 0:
                sign = -1

            offset_ratio = max(min(current_offset/self.initial_offset, 1), -1)
            # self.power = abs(max(min(self.power*(offset_ratio), self.max_power), self.min_power)) * sign
            self.power = abs(max(min(self.power, self.max_power), self.min_power)) * sign

            print("POWER: ", self.power)
            print("INITIAL OFFSET:", self.initial_offset, "CURRENT OFFSET: ", current_offset)
            print("OFFSET RATIO: ", offset_ratio)

            self.subsystem.m_turret.set_raw_output(self.power)

        elif current_offset:
            self.new_run = True
            self.initial_offset = 1
            true_angle = Robot.limelight.k_cam_angle + Robot.limelight.table.getNumber('ty', None)
            distance = (Robot.limelight.k_h_hub_height - Robot.limelight.k_cam_height) / math.tan(true_angle)
            # self.subsystem.target_stationary(distance)
            self.subsystem.m_turret.set_raw_output(0)
        else:
            self.new_run = True
            self.initial_offset = 1
            self.subsystem.stop()
            self.subsystem.m_turret.set_raw_output(0)

        # self.old_limelight = Robot.limelight.get_x_offset()

        # d_theta = self.subsystem.get_turret_rotation_velocity()
        # d_current = Robot.limelight.get_x_offset()
        # d_omega = self.old_limelight - d_current

        # d_theta = d_theta*d_current/d_omega
        #
        # if abs(self.subsystem.get_turret_rotation_velocity()) < .1 and Robot.limelight.get_x_offset() != 0:
        #     self.c_count += 1
        #     if self.c_count >= self.ready_counts:
        #         self.subsystem.ready = True
        # else:
        #     self.c_count = 0
        #     self.subsystem.ready = False
        #
        # print(d_theta)

        # self.subsystem.m_turret.set_raw_output(d_theta)

    def end(self, interrupted: bool) -> None:
        Robot.shooter.ready = False
        Robot.limelight.ref_off()

    def isFinished(self) -> bool:
        return False


class NaiveDemoShot(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter):
        super().__init__(subsystem)

    def initialize(self) -> None:
        self.subsystem.set_launch_angle(.2)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()
