import math

import commands2
from robotpy_toolkit_7407.command import SubsystemCommand
from wpimath._controls._controls.controller import ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import TrapezoidProfileRadians

import constants
from commands.drivetrain.constants import AIM_kP, AIM_kI, AIM_kD, AIM_max_angular_vel, AIM_max_angular_accel
from commands.drivetrain.drive_swerve_custom import DriveSwerveCustom
from robot_systems import Robot
from subsystem import Drivetrain


class DriveSwerveTurretAim(SubsystemCommand[Drivetrain]):

    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)

        self.pid_controller = ProfiledPIDControllerRadians(
            AIM_kP, AIM_kI, AIM_kD,
            TrapezoidProfileRadians.Constraints(
                AIM_max_angular_vel, AIM_max_angular_accel
            ), constants.period
        )

        self.ready = False

    def initialize(self) -> None:
        self.ready = False
        Robot.shooter.aiming = True

    def execute(self) -> None:
        print("Running DriveSwerveTurretAim.")

        dx, dy = Robot.drivetrain.axis_dx.value, Robot.drivetrain.axis_dy.value
        current_limelight_offset = Robot.limelight.table.getNumber('tx', None)

        # wpilib.SmartDashboard.putNumber("I want to go to:", Robot.shooter.desired_turret_angle)

        if current_limelight_offset is not None and current_limelight_offset != 0 and abs(current_limelight_offset < 2):
            self.ready = True
        elif current_limelight_offset < -2:
            Robot.drivetrain.set((dx, dy), 3)
        elif current_limelight_offset > 2:
            Robot.drivetrain.set((dx, dy), -3)
        elif (Robot.shooter.desired_turret_angle - math.degrees(Robot.shooter.get_turret_rotation_angle())) > 5:
            Robot.drivetrain.set((dx, dy), -6)
        elif (Robot.shooter.desired_turret_angle - math.degrees(Robot.shooter.get_turret_rotation_angle())) < -5:
            Robot.drivetrain.set((dx, dy), 6)
        else:
            self.ready = True

        # hub_angle = Robot.shooter.desired_turret_angle
        # current_limelight_offset = Robot.limelight.table.getNumber('tx', None)
        #
        #
        # if hub_angle is not None:
        #     omega = self.pid_controller.calculate(hub_angle, 0)
        #     print("OMEGA", omega)
        #
        #     dx *= constants.drivetrain_target_max_vel
        #     dy *= -constants.drivetrain_target_max_vel
        #
        #     Robot.drivetrain.set((dx, dy), omega)
        #
        # if current_limelight_offset is not None and current_limelight_offset != 0 and current_limelight_offset < 2:
        #     self.ready = True

    def end(self, interrupted: bool) -> None:
        print("DONE AIMING!!")
        Robot.shooter.aiming = False
        commands2.CommandScheduler.getInstance().schedule(DriveSwerveCustom(Robot.drivetrain))

    def isFinished(self) -> bool:
        return self.ready

    def runsWhenDisabled(self) -> bool:
        return False
