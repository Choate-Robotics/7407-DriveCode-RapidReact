from networktables import NetworkTables
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit
from robotpy_toolkit_7407.utils.units import deg, rad, s, m

from robot_systems import Robot
from subsystem import Shooter


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
        self.old_limelight = None
        self.ready_counts = ready_counts
        self.c_count = 0

    def initialize(self) -> None:
        self.old_limelight = Robot.limelight.get_x_offset().asNumber(deg)

    def execute(self) -> None:
        self.old_limelight = Robot.limelight.get_x_offset().asNumber(deg)

        d_theta = self.subsystem.get_turret_rotation_velocity().asNumber(m / s)
        d_current = Robot.limelight.get_x_offset().asNumber(deg)
        d_omega = self.old_limelight - d_current

        d_theta = d_theta*d_current/d_omega

        if abs(self.subsystem.get_turret_rotation_velocity()) < .1 and Robot.limelight.get_x_offset().asNumber(rad) != 0:
            self.c_count += 1
            if self.c_count >= self.ready_counts:
                self.subsystem.ready = True
        else:
            self.c_count = 0
            self.subsystem.ready = False

        self.subsystem.set_turret_rotation_velocity(d_theta)

    def end(self, interrupted: bool) -> None:
        Robot.shooter.ready = False
        Robot.limelight.ref_off()

    def isFinished(self) -> bool:
        return False
