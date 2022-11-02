import time
from ast import Sub
import robotpy_toolkit_7407.subsystem_templates.drivetrain.swerve_drivetrain_commands
import wpilib
from networktables import NetworkTables
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit
from robotpy_toolkit_7407.utils.units import deg, rad, s, m

import oi.keymap
import robot_systems
from robot_systems import Robot
from subsystem import Shooter

import math

from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
import constants

from command.drivetrain import DriveSwerveCustom
import commands2

import subsystem


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


class TurretZero(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter):
        super().__init__(subsystem)

    def initialize(self) -> None:
        if not self.subsystem.turret_zeroed:
            self.subsystem.m_turret.set_raw_output(-.15)

    def execute(self) -> None:
        if not self.subsystem.turret_zeroed:
            if self.subsystem.mag_sensor.get_value():
                self.subsystem.m_turret.set_raw_output(0)
                self.subsystem.m_turret.set_sensor_position(0)
                self.subsystem.turret_zeroed = True
                print("YES TURRET IS ZEROED!!!!")

    def isFinished(self) -> bool:
        return self.subsystem.turret_zeroed


class TurretAim(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter):
        super().__init__(subsystem)
        self.target_angle = None
        self.target_dist = None
        self.target_power = None

        self.turret_min_absolute_power = constants.min_turret_power  # Minimum power of turret required to move it
        self.turret_max_absolute_power = constants.max_turret_power  # Maximum movement power of turret allowed .80

        # Soft limits for turret movement
        self.limit_backward = False
        self.limit_forward = False

        self.turret_default_movement_power = constants.default_turret_power  # Default movement power of turret if not using pid

        self.turret_kP = .02  # Multiplies current offset by this multiplier to get power .0275

        self.turret_min_angle = 0 + 10  # Minimum angle of turret range
        self.turret_max_angle = self.subsystem.turret_max_angle - 10  # Maximum angle of turret range

        self.limelight_angle_threshold = 2  # Angle threshold for limelight to be considered on target

        self.limelight_detected_counts = 0  # Counts how many times limelight has detected target in a row

        self.current_turret_angle = 0

        self.ejecting = False
        self.ejection_allowed_time = .25
        self.start_ejection_time = time.time()
        self.last_ball = "red"

        self.ejection_multiplier = 2

        self.current_offset = 100

    def initialize(self) -> None:
        pass

    def is_shooter_ready(self):
        m_top_desired = self.subsystem.desired_m_top
        m_bottom_desired = self.subsystem.desired_m_bottom

        # If the robot is ejecting, it says that it's ready to shoot with more room for error.

        if self.ejecting:
            return (
                    abs(Robot.shooter.m_bottom.get_sensor_velocity() - m_bottom_desired) < .05 * m_bottom_desired
                    and abs(Robot.shooter.m_top.get_sensor_velocity() - m_top_desired) < .05 * m_top_desired
            )

        return (
                self.current_offset < self.limelight_angle_threshold
                and abs(Robot.shooter.m_top.get_sensor_velocity() - m_top_desired) < .05 * m_top_desired
                and abs(Robot.shooter.m_bottom.get_sensor_velocity() - m_bottom_desired) < .05 * m_bottom_desired
                and abs(Robot.drivetrain.chassis_speeds.omega) < 0.1
        )

    def set_ejection(self):
        if robot_systems.Sensors.new_color_sensor.get_value():
            ball_color = "red"
        elif Robot.index.photo_electric.get_value():
            ball_color = "blue"
        else:
            ball_color = constants.team_color

        if ball_color != constants.team_color:
            self.start_ejection_time = time.time()
            self.ejecting = True
        elif (time.time() - self.start_ejection_time) > self.ejection_allowed_time:
            self.ejecting = False

    def set_aim_dist(self):
        def get_limelight_dist():
            est_ty = Robot.limelight.table.getNumber('ty', None)
            true_angle = Robot.limelight.k_cam_angle + math.radians(est_ty)
            distance = (Robot.limelight.k_h_hub_height - Robot.limelight.k_cam_height) / math.tan(true_angle)

            return distance

        self.target_dist = None

        if self.current_offset is not None and self.current_offset != 0:
            self.subsystem.seen_after_drivetrain_rezero = True
            self.target_dist = get_limelight_dist() * (self.ejection_multiplier if self.ejecting else 1)

        else:
            if Robot.odometry.hub_dist is not None:
                self.target_dist = Robot.odometry.hub_dist * (self.ejection_multiplier if self.ejecting else 1)
            elif self.ejecting:
                self.target_dist = .1

        if self.subsystem.target_turret_dist is not None:
            self.target_dist = self.subsystem.target_turret_dist

    def set_aim_power(self):
        def limit_angles():
            current_angle = math.degrees(self.subsystem.get_turret_rotation_angle())

            if current_angle <= self.turret_min_angle:
                self.limit_backward = True
            else:
                self.limit_backward = False

            if current_angle >= self.turret_max_angle:
                self.limit_forward = True
            else:
                self.limit_forward = False

        self.target_power = None
        limit_angles()

        if self.current_offset is not None and abs(self.current_offset) > self.limelight_angle_threshold:
            self.target_power = abs(self.current_offset * self.turret_kP)

            if self.current_offset > 0:
                if self.limit_forward:
                    sign = 0
                else:
                    sign = 1
            elif self.current_offset < 0:
                if self.limit_backward:
                    sign = 0
                else:
                    sign = -1
            else:
                sign = 0

            self.target_power = abs(
                max(min(self.target_power, self.turret_max_absolute_power), self.turret_min_absolute_power)) * sign
            if self.target_power == 0:
                self.target_power = None

        if self.subsystem.target_turret_angle is not None:
            self.target_power = None

    def set_aim_angle(self):
        def get_desired_turret_angle():
            hub_angle = math.degrees(Robot.odometry.hub_angle)
            if hub_angle >= 0:
                desired_angle = self.current_turret_angle + hub_angle
            else:
                desired_angle = self.current_turret_angle + hub_angle + 360

            desired_angle %= 360

            return desired_angle

        self.target_angle = None

        if self.current_offset == 0 or self.current_offset is None:
            if self.current_turret_angle is None:
                self.current_turret_angle = math.degrees(self.subsystem.get_turret_rotation_angle())

            self.target_angle = get_desired_turret_angle()

            if Robot.odometry.hub_angle is not None:
                pass
        else:
            self.current_turret_angle = math.degrees(self.subsystem.get_turret_rotation_angle())

        if self.subsystem.target_turret_angle is not None:
            self.target_angle = self.subsystem.target_turret_angle

        if self.target_angle is not None:
            self.target_angle = math.radians(max(min(self.target_angle, self.turret_max_angle), self.turret_min_angle))

    def set_zero(self):
        if not self.subsystem.turret_zeroed:
            self.target_power = -.2
            self.target_angle = None
            self.subsystem.turret_zeroed = self.subsystem.mag_sensor.get_value()
            if self.subsystem.turret_zeroed:
                self.subsystem.m_turret.set_sensor_position(0)
                self.target_power = None

    def execute(self) -> None:
        if self.subsystem.auto_finished:
            Robot.odometry.update()

        self.current_offset = Robot.limelight.table.getNumber('tx', None)
        self.set_ejection()
        self.set_aim_dist()
        self.set_aim_power()
        self.set_aim_angle()
        self.set_zero()

        if self.target_angle is not None:
            self.subsystem.set_turret_angle(self.target_angle)
        elif self.target_power is not None:
            self.subsystem.m_turret.set_raw_output(self.target_power)
        else:
            self.subsystem.m_turret.set_raw_output(0)

        if self.target_dist is not None:
            self.subsystem.target_stationary(self.target_dist)
        else:
            self.subsystem.stop()

        self.subsystem.ready = self.is_shooter_ready()

        # Logging
        wpilib.SmartDashboard.putBoolean("EJECTING", self.ejecting)

    def end(self, interrupted: bool) -> None:
        self.subsystem.ready = False
        Robot.limelight.ref_off()
        self.subsystem.stop()
        self.subsystem.m_turret.set_raw_output(0)

    def isFinished(self) -> bool:
        return False


class TurretDriveAim(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter):
        super().__init__(subsystem)

    def initialize(self) -> None:
        self.subsystem.set_turret_angle(math.radians(125))
        self.subsystem.target_stationary(Robot.odometry.hub_dist)

    def execute(self) -> None:
        print("Running TurretDriveAim")
        pass

    def isFinished(self) -> bool:
        return not self.subsystem.aiming

    def end(self, interrupted: bool) -> None:
        print("STOPPED AIMING TURRET")
        commands2.CommandScheduler.getInstance().schedule(TurretAim(Robot.shooter))


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
