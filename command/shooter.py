from ast import Sub
import robotpy_toolkit_7407.subsystem_templates.drivetrain.swerve_drivetrain_commands
import wpilib
from networktables import NetworkTables
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit
from robotpy_toolkit_7407.utils.units import deg, rad, s, m

import oi.keymap
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


class TurretAim(SubsystemCommand[Shooter]):
    def __init__(self, subsystem: Shooter):
        super().__init__(subsystem)
        self.power = 0

        self.min_absolute_power = .07  # Minimum power of turret required to move it
        self.max_absolute_power = .40  # Maximum movement power of turret allowed .80

        # Soft limits for turret movement
        self.limit_backward = False
        self.limit_forward = False

        self.default_movement_power = .20  # Default movement power of turret if not using pid

        self.p = .02  # Multiplies current offset by this multiplier to get power .0275

        self.min_angle = 0 + 10  # Minimum angle of turret range
        self.max_angle = self.subsystem.turret_max_angle - 10  # Maximum angle of turret range

        self.limelight_angle_threshold = 2  # Angle threshold for limelight to be considered on target

        self.limelight_detected_counts = 0  # Counts how many times limelight has detected target in a row

        self.current_shooter_angle = 0

    def initialize(self) -> None:
        pass

    def execute(self) -> None:

        print("ROBOT SHOOTER TARGET DIST IS: ", Robot.shooter.target_turret_dist)
        print("ROBOT SHOOTER TARGET ANGLE IS: ", Robot.shooter.target_turret_angle)

        if self.subsystem.turret_zeroed:
            # Soft limit based on turret range
            current_angle = math.degrees(self.subsystem.get_turret_rotation_angle())
            wpilib.SmartDashboard.putNumber("current_angle", current_angle)
            wpilib.SmartDashboard.putNumber("limit_backward", self.limit_backward)
            wpilib.SmartDashboard.putNumber("limit_forward", self.limit_forward)
            wpilib.SmartDashboard.putNumber("detected_counts", self.limelight_detected_counts)
            wpilib.SmartDashboard.putBoolean("Shooter Ready", self.subsystem.ready)

            if self.limelight_detected_counts < 3:
                self.subsystem.stop()

            if current_angle <= self.min_angle:
                self.limit_backward = True
            else:
                self.limit_backward = False

            if current_angle >= self.max_angle:
                self.limit_forward = True
            else:
                self.limit_forward = False

            current_offset = Robot.limelight.table.getNumber('tx', None)

            # Estimating stuff

            if current_offset is not None and current_offset != 0:
                self.current_shooter_angle = None
                Robot.odometry.update()

                self.limelight_detected_counts += 1

                est_ty = Robot.limelight.table.getNumber('ty', None)
                true_angle = Robot.limelight.k_cam_angle + math.radians(est_ty)
                distance = (Robot.limelight.k_h_hub_height - Robot.limelight.k_cam_height) / math.tan(
                    true_angle) + .5  # constant for lower turret
                wpilib.SmartDashboard.putNumber("Distance to Hub", distance)
                if self.limelight_detected_counts >= 3:
                    if self.subsystem.target_turret_dist is None:
                        self.subsystem.target_stationary(distance)
                    else:
                        self.subsystem.target_stationary(self.subsystem.target_turret_dist)
                else:
                    self.subsystem.stop()

                if abs(current_offset) > self.limelight_angle_threshold:
                    self.subsystem.ready = False

                    self.power = abs(current_offset * self.p)

                    if current_offset > 0:
                        if self.limit_forward:
                            sign = 0
                        else:
                            sign = 1
                    elif current_offset < 0:
                        if self.limit_backward:
                            sign = 0
                        else:
                            sign = -1
                    else:
                        sign = 0

                    self.power = abs(max(min(self.power, self.max_absolute_power), self.min_absolute_power)) * sign

                    if self.subsystem.target_turret_angle is not None:
                        desired_turret_angle = math.degrees(self.subsystem.target_turret_angle)
                        Robot.shooter.set_turret_angle(
                            math.radians(max(min(desired_turret_angle, self.max_angle), self.min_angle))
                        )
                    else:
                        self.subsystem.m_turret.set_raw_output(self.power)

                    wpilib.SmartDashboard.putNumber("power", self.power)

                else:
                    self.subsystem.m_turret.set_raw_output(0)
                    if self.limelight_detected_counts >= 6 and abs(Robot.drivetrain.chassis_speeds.omega) < 0.1:
                        self.subsystem.ready = True
                    else:
                        self.subsystem.ready = False

            else:
                self.limelight_detected_counts = 0

                if self.current_shooter_angle is None:
                    self.current_shooter_angle = self.subsystem.get_turret_rotation_angle()

                Robot.odometry.update()

                if Robot.odometry.hub_angle is not None:
                    if Robot.odometry.hub_dist is not None and self.subsystem.target_turret_dist is None:
                        self.subsystem.target_stationary(Robot.odometry.hub_dist)

                    hub_angle = math.degrees(Robot.odometry.hub_angle)
                    current_shooter_angle = math.degrees(self.current_shooter_angle)

                    if hub_angle >= 0:
                        desired_turret_angle = current_shooter_angle + hub_angle
                    else:
                        desired_turret_angle = current_shooter_angle + hub_angle + 360

                    desired_turret_angle %= 360

                    self.subsystem.desired_turret_angle = desired_turret_angle

                    # print("DESIRED TURRET ANGLE: ", desired_turret_angle)
                    wpilib.SmartDashboard.putNumber("DESIRED_TURRET_ANGLE", desired_turret_angle)
                    wpilib.SmartDashboard.putNumber("CURRENT_TURRET_ANGLE", current_shooter_angle)

                    if self.subsystem.target_turret_angle is not None:
                        desired_turret_angle = math.degrees(self.subsystem.target_turret_angle)

                    Robot.shooter.set_turret_angle(
                        math.radians(max(min(desired_turret_angle, self.max_angle), self.min_angle))
                    )

                    if abs(math.degrees(self.subsystem.get_turret_rotation_angle()) - math.degrees(
                            Robot.odometry.hub_angle)) < 3:
                        self.subsystem.ready = True
                    else:
                        self.subsystem.ready = False

                if self.subsystem.target_turret_dist is not None:
                    self.subsystem.target_stationary(self.subsystem.target_turret_dist)

        else:
            self.subsystem.m_turret.set_raw_output(-.15)
            self.subsystem.turret_zeroed = self.subsystem.mag_sensor.get_value()
            if self.subsystem.turret_zeroed:
                self.subsystem.m_turret.set_raw_output(0)
                self.subsystem.m_turret.set_sensor_position(0)

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
