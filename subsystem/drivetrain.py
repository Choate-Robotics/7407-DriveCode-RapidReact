import wpilib
import wpilib.kinematics
from wpimath.geometry import Rotation2d
import ctre
import commands2 as commands

import utils.logger as logger
from robot_lib.motor import PIDMotor
from robot_lib.motors.ctre_motors import TalonGroup, TalonFX, TalonConfig
from robot_lib.subsystem import Subsystem
from utils.math import sensor_units_to_meters

import sensors
from utils.network import Network


class Drivetrain(Subsystem):
    _config: TalonConfig = TalonConfig(0.5, 0, 0, 1, 15000, 10000)
    m_left: PIDMotor = TalonGroup(TalonFX(0), TalonFX(1), TalonFX(2), config=TalonConfig(0.5, 0, 0, 1, 15000, 10000))
    m_right: PIDMotor = TalonGroup(TalonFX(3), TalonFX(4), TalonFX(5), config=TalonConfig(0.5, 0, 0, 1, 15000, 10000))
    flipped: bool

    def init(self) -> None:
        logger.info("initializing drivetrain", "[drivetrain]")

        self.m_left.init()
        self.m_right.init()
        self.flipped = False

        logger.info("initialization complete", "[drivetrain]")

    def set_motor_percent_output(self, left: float, right: float):
        if self.flipped:
            left, right = right, left
        self.m_left.set_raw_output(left)
        self.m_right.set_raw_output(right)

    def set_motor_velocity(self, left: float, right: float):
        if self.flipped:
            left, right = right, left
        self.m_left.set_target_velocity(left)
        self.m_right.set_target_velocity(right)

    def reset_pose(self, flipped: bool = False):
        # logger.info(f"resetting pose{' (flipped)' if flipped else ''}....")
        # self.flipped = flipped
        # self.gyro.reset(0)
        # self.odometry = wpilib.kinematics.DifferentialDriveOdometry(Rotation2d(0))
        # self.left1.setSelectedSensorPosition(0)
        # self.right1.setSelectedSensorPosition(0)
        pass

    def get_pose(self):
        # return self.odometry.getPose()
        pass
