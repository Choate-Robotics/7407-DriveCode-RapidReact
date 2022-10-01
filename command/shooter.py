import wpilib
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
        self.power = 0

        self.min_absolute_power = .07
        self.max_absolute_power = .80

        self.limit_backward = False
        self.limit_forward = False

        self.default_movement_power = .20

        self.p = .0275  # Multiplies current offset by this multiplier to get power .03
        self.i = 0
        self.d = 0

        self.min_angle = 0
        self.max_angle = self.subsystem.turret_max_angle

        self.limelight_angle_threshold = 2

        self.limelight_detected_counts = 0

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        # Soft limit based on turret range
        current_angle = math.degrees(Robot.shooter.get_turret_rotation_angle())
        wpilib.SmartDashboard.putNumber("current_angle", current_angle)
        wpilib.SmartDashboard.putNumber("limit_backward", self.limit_backward)
        wpilib.SmartDashboard.putNumber("limit_forward", self.limit_forward)

        wpilib.SmartDashboard.putNumber("detected_counts", self.limelight_detected_counts)

        wpilib.SmartDashboard.putBoolean("Shooter Ready", Robot.shooter.ready)

        if self.limelight_detected_counts < 3:
            self.subsystem.stop()

        if current_angle <= (self.min_angle + 10):
            self.limit_backward = True
        else:
            self.limit_backward = False

        if current_angle >= (self.max_angle - 10):
            self.limit_forward = True
        else:
            self.limit_forward = False

        current_offset = Robot.limelight.table.getNumber('tx', None)

        if current_offset is not None:

            if current_offset == 0:
                self.limelight_detected_counts = 0

            self.limelight_detected_counts += 1

            est_ty = Robot.limelight.table.getNumber('ty', None)
            true_angle = Robot.limelight.k_cam_angle + math.radians(est_ty)
            distance = (Robot.limelight.k_h_hub_height - Robot.limelight.k_cam_height) / math.tan(true_angle)

            if self.limelight_detected_counts >= 3:
                self.subsystem.target_stationary(distance)
            else:
                self.subsystem.stop()

            wpilib.SmartDashboard.putNumber("current_offset", current_offset)
            wpilib.SmartDashboard.putNumber("pid power", current_offset * self.p)
            # wpilib.SmartDashboard.putNumber("cam_angle", Robot.limelight.k_cam_angle)
            # wpilib.SmartDashboard.putNumber("est_ty", est_ty)
            # wpilib.SmartDashboard.putNumber("est_ty_degrees", math.radians(est_ty))
            # wpilib.SmartDashboard.putNumber("true_angle", true_angle)
            # wpilib.SmartDashboard.putNumber("distance", distance)

            if abs(current_offset) > self.limelight_angle_threshold:

                Robot.shooter.ready = False

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

                self.subsystem.m_turret.set_raw_output(self.power)
                wpilib.SmartDashboard.putNumber("power", self.power)

            else:
                self.subsystem.m_turret.set_raw_output(0)
                if self.limelight_detected_counts >= 6 and abs(Robot.drivetrain.chassis_speeds.omega) < 0.1:
                    Robot.shooter.ready = True
                else:
                    Robot.shooter.ready = False

        else:
            self.subsystem.stop()
            self.subsystem.m_turret.set_raw_output(0)
            Robot.shooter.ready = False

    def end(self, interrupted: bool) -> None:
        Robot.shooter.ready = False
        Robot.limelight.ref_off()
        Robot.shooter.stop()
        Robot.shooter.m_turret.set_raw_output(0)

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
