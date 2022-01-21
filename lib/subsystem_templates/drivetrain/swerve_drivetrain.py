import math

from lib.oi.joysticks import JoystickAxis
from lib.subsystem import Subsystem
from utils import logger
from utils.math import rotate_vector


class SwerveNode:
    motor_reversed: bool
    motor_flip_diff: float

    def init(self):
        self.motor_reversed = False
        self.motor_flip_diff = 0

    def set(self, vel_tw_per_second: float, angle_radians: float):
        self.set_angle_radians(angle_radians, self.get_current_angle_raw() + self.motor_flip_diff)
        self.set_velocity_raw(vel_tw_per_second)

    # pos=0 - facing right, counter-clockwise around, pos=1 - facing right
    def set_angle_raw(self, pos: float): ...

    def get_current_angle_raw(self) -> float: ...

    # 0 degrees is facing right
    def set_angle_radians(self, target_radians: float, initial_radians: float):
        # Actual angle difference in radians
        diff = math.fmod(target_radians, 2 * math.pi) - math.fmod(initial_radians, 2 * math.pi)
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi

        # Should we flip the motor
        if abs(diff) > math.pi / 2:
            if diff < 0:
                self.motor_flip_diff += math.pi
                diff += math.pi
            else:
                self.motor_flip_diff -= math.pi
                diff -= math.pi
            self.motor_reversed = not self.motor_reversed

        # Add diff to theta f
        theta_f = initial_radians + diff

        self.set_angle_raw(theta_f - self.motor_flip_diff)

    def set_velocity_raw(self, vel_tw_per_second: float): ...

    def get_current_velocity(self) -> float: ...


class SwerveOdometry:
    def init(self): ...
    def get_robot_angle_degrees(self) -> float: ...
    def reset_angle(self): ...


class SwerveDrivetrain(Subsystem):
    n_00: SwerveNode  # Top Left
    n_01: SwerveNode  # Bottom Left
    n_10: SwerveNode  # Top Right
    n_11: SwerveNode  # Bottom Right
    odometry: SwerveOdometry
    axis_dx: JoystickAxis
    axis_dy: JoystickAxis
    axis_rotation: JoystickAxis
    track_width: float

    def init(self):
        logger.info("initializing swerve drivetrain", "[swerve_drivetrain]")
        self.n_00.init()
        self.n_01.init()
        self.n_10.init()
        self.n_11.init()
        self.odometry.init()
        logger.info("initialization complete", "[swerve_drivetrain]")

    def set(self, vel_tw_per_second: tuple[float, float], angular_vel: float):
        vel_tw_per_second = rotate_vector(
            vel_tw_per_second[0],
            vel_tw_per_second[1],
            -self.odometry.get_robot_angle_degrees() * (math.pi / 180)
        )

        # logger.info(f"ROBOT AT {self.odometry.get_robot_angle_degrees()} degrees offset")
        # logger.info(f"({vel_tw_per_second[0]} tw/sec, {vel_tw_per_second[1]} tw/sec, {angular_vel} rad/sec)")

        if abs(vel_tw_per_second[0]) < 0.1 and abs(vel_tw_per_second[1]) < 0.1 and abs(angular_vel) < 0.1:
            self.n_00.set_velocity_raw(0)
            self.n_01.set_velocity_raw(0)
            self.n_10.set_velocity_raw(0)
            self.n_11.set_velocity_raw(0)
            return

        self.n_00.set(*self._swerve_displacement(-1, -1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel, 0))
        self.n_01.set(*self._swerve_displacement(-1, 1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel, math.pi))
        self.n_10.set(*self._swerve_displacement(1, -1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel, math.pi))
        self.n_11.set(*self._swerve_displacement(1, 1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel, 0))

    def stop(self):
        self.n_00.set(0, 0)
        self.n_01.set(0, 0)
        self.n_10.set(0, 0)
        self.n_11.set(0, 0)

    @staticmethod
    def _swerve_displacement(node_x: float, node_y: float, dx: float, dy: float, d_theta: float, angle_offset: float) -> tuple[float, float]:
        tangent_x, tangent_y = -node_y, node_x
        tangent_m = math.sqrt(tangent_x*tangent_x + tangent_y*tangent_y)
        tangent_x /= tangent_m
        tangent_y /= tangent_m

        r = math.sqrt(2) / 2
        sx = dx + r * d_theta * tangent_x
        sy = dy + r * d_theta * tangent_y

        theta = math.atan2(sy, sx)
        magnitude = math.sqrt(sx * sx + sy * sy)
        return magnitude, theta + angle_offset
