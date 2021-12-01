import math

from lib.oi.joysticks import JoystickAxis
from lib.subsystem import Subsystem
from utils import logger


class SwerveNode:
    def __init__(self):
        self._old_radians = 0

    def init(self): ...

    def set(self, vel_tw_per_second: float, angle_radians: float):
        self.set_angle_radians(angle_radians, self._old_radians)
        self.set_velocity_raw(vel_tw_per_second)
        self._old_radians = angle_radians

    # pos=0 - facing right, counter-clockwise around, pos=1 - facing right
    def set_angle_raw(self, pos: float): ...

    # 0 degrees is facing right
    def set_angle_radians(self, pos_radians: float, old_radians: float):
        self.set_angle_raw((pos_radians / 360) % 1)

    def set_velocity_raw(self, vel_tw_per_second: float): ...


class SwerveOdometry:
    def init(self): ...
    def get_robot_angle_degrees(self) -> float: ...


class SwerveDrivetrain(Subsystem):
    n_00: SwerveNode  # Top Left
    n_01: SwerveNode  # Bottom Left
    n_10: SwerveNode  # Top Right
    n_11: SwerveNode  # Bottom Right
    odometry: SwerveOdometry
    axis_dx: JoystickAxis
    axis_dy: JoystickAxis
    axis_rotation: JoystickAxis

    def init(self):
        logger.info("initializing swerve drivetrain", "[swerve_drivetrain]")
        self.n_00.init()
        self.n_01.init()
        self.n_10.init()
        self.n_11.init()
        self.odometry.init()
        logger.info("initialization complete", "[swerve_drivetrain]")

    def set(self, vel_tw_per_second: tuple[float, float], angular_vel: float):
        self.n_00.set(*self._swerve_displacement(-1, -1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel))
        self.n_01.set(*self._swerve_displacement(-1, 1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel))
        self.n_10.set(*self._swerve_displacement(1, -1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel))
        self.n_11.set(*self._swerve_displacement(1, 1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel))

    def stop(self):
        self.n_00.set(0, 0)
        self.n_01.set(0, 0)
        self.n_10.set(0, 0)
        self.n_11.set(0, 0)

    @staticmethod
    def _swerve_displacement(node_x: float, node_y: float, dx: float, dy: float, d_theta: float) -> tuple[float, float]:
        sin_theta = math.sin(d_theta)
        cos_theta = math.cos(d_theta)
        sx = dx + node_x * cos_theta - node_y * sin_theta - node_x
        sy = dy + node_x * sin_theta + node_y * cos_theta - node_y
        theta = math.atan2(sy, sx)
        magnitude = math.sqrt(sx * sx + sy * sy) / 2
        return magnitude, theta
