from lib.subsystem import Subsystem
from utils import logger


class SwerveNode:
    def init(self): ...

    # pos=0 - facing right, counter-clockwise around, pos=1 - facing right
    def set_position_raw(self, pos: float): ...

    # 0 degrees is facing right
    def set_position_degrees(self, pos_degrees: float, old_degrees: float):
        self.set_position_raw((pos_degrees / 360) % 1)

    def set_velocity_raw(self, vel: float): ...


class SwerveOdometry:
    def init(self): ...
    def get_robot_angle_degrees(self) -> float: ...


class SwerveDrivetrain(Subsystem):
    n_00: SwerveNode  # Top Left
    n_01: SwerveNode  # Bottom Left
    n_10: SwerveNode  # Top Right
    n_11: SwerveNode  # Bottom Right
    odometry: SwerveOdometry

    def init(self):
        logger.info("initializing swerve drivetrain", "[swerve_drivetrain]")
        self.n_00.init()
        self.n_01.init()
        self.n_10.init()
        self.n_11.init()
        self.odometry.init()
        logger.info("initialization complete", "[swerve_drivetrain]")