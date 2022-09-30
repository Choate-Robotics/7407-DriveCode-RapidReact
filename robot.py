# HARTFORD READY

import commands2
import wpilib
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.utils import logger
from wpimath.geometry import Pose2d

import config
import constants
from autonomous import two_ball_auto, five_ball_auto, five_ball_auto_red
from autonomous.auto_routine import AutoRoutine
# from autonomous import five_ball_auto, two_ball_auto, three_ball_auto # TODO: Fix this
from command import BallPath
from command import ElevatorRezero
from command import TurretAim
from command.drivetrain import DriveSwerveCustom
from oi.OI import OI
from robot_systems import Robot, Pneumatics, Sensors
from sensors.color_sensors import ColorSensors
from sensors.field_odometry import FieldOdometry
from sensors.intake_cameras import IntakeCameras
from sensors.limelight import Limelight
from sensors.rev_digit import RevDigit

from robotpy_toolkit_7407.utils.units import rad, deg, radians, meters_per_second, m, s

import math


# from sensors.intake_cameras import IntakeCameras


class _Robot(wpilib.TimedRobot):
    """
    Main robot class. Initializes OI and subsystems, and runs the command scheduler.
    """

    def __init__(self):
        super().__init__(constants.period)

        # self.auto_routines = [
        #     two_ball_auto.routine,
        #     three_ball_auto.routine,
        #     five_ball_auto.routine
        # ] # TODO Fix This
        self.auto_routines = [two_ball_auto.routine, five_ball_auto.routine]

        self.auto_routine: AutoRoutine | None = None
        self.initial_pose: Pose2d | None = None
        self.auto_combo = "CALL SID"

        self.button_1_last = None
        self.button_2_last = None
        self.emergency = True

    def robotInit(self):
        wpilib.LiveWindow.disableAllTelemetry()  # Disable Telemetry on Robot Startup to reduce loop time
        """
        Called on robot startup. Here the subsystems and oi are all initialized.
        """
        if self.isReal():
            logger.Color = logger.NoColor  # Disable log colors when on the robot

        logger.info("initializing robot")

        subsystems: list[Subsystem] = list(
            {k: v for k, v in Robot.__dict__.items() if isinstance(v, Subsystem)}.values()
        )

        for subsystem in subsystems:
            subsystem.init()

        # OI
        OI.init()
        OI.map_controls()

        Robot.rev_digit = RevDigit()

        # Pneumatics
        Pneumatics.compressor.enableAnalog(90, 120)

        Sensors.color_sensors = ColorSensors()

        commands2.CommandScheduler.getInstance().setPeriod(constants.period)

        # Robot.intake_cameras = IntakeCameras(Robot.intake)  # TODO

        Robot.odometry = FieldOdometry(Robot.drivetrain)

        Robot.limelight = Limelight()

        self.auto_routine = five_ball_auto_red.routine
        self.auto_combo = "Five RED"
        self.emergency = False

        self.initial_pose = self.auto_routine.initial_robot_pose

        Robot.drivetrain.gyro._gyro.setYaw(
            self.initial_pose.rotation().degrees()
        )
        Robot.drivetrain.odometry.resetPosition(self.initial_pose, self.initial_pose.rotation())

        logger.info("initialization complete")

    def robotPeriodic(self):
        # print(Robot.drivetrain.odometry.getPose())
        Robot.rev_digit.update()
        commands2.CommandScheduler.getInstance().run()

        wpilib.SmartDashboard.putString('DB/String 0', f'Team Color: {config.TEAM}')
        wpilib.SmartDashboard.putString('DB/String 1', f'CALL 4 PRGMER HELP <3?: {self.emergency}')
        wpilib.SmartDashboard.putString('DB/String 2',
                                        f'Compressor Value: {round(Pneumatics.compressor.getPressure(), 1)}')
        wpilib.SmartDashboard.putString('DB/String 3',
                                        f'D_Motor Temp (C): {(Robot.drivetrain.n_00.m_move._motor.getTemperature() + Robot.drivetrain.n_01.m_move._motor.getTemperature() + Robot.drivetrain.n_10.m_move._motor.getTemperature() + Robot.drivetrain.n_11.m_move._motor.getTemperature()) / 4}')
        # wpilib.SmartDashboard.putString('DB/String 5', f'Color Sensors: {"WORKING" if Sensors.color_sensors.working == True else "FAILED" if not Sensors.color_sensors.working else Sensors.color_sensors.working}')
        wpilib.SmartDashboard.putString('DB/String 6', f'EJECTION_ON: {config.EJECT_ENABLE}')
        wpilib.SmartDashboard.putString('DB/String 7', f'AUTO MODE: {self.auto_combo}')

        color_status = 'WORKING'
        if not Sensors.color_sensors.working:
            color_status = 'FAILED'
        wpilib.SmartDashboard.putString('DB/String 5', f'Color Sens: {color_status}')

        if self.button_1_last is None:
            self.button_1_last = wpilib.SmartDashboard.getBoolean('DB/Button 1', False)

        if self.button_2_last is None:
            self.button_2_last = wpilib.SmartDashboard.getBoolean('DB/Button 2', False)

        if wpilib.SmartDashboard.getBoolean('DB/Button 1', self.button_1_last) != self.button_1_last:
            self.button_1_last = not self.button_1_last
            if config.TEAM == "red":
                config.TEAM = "blue"
            else:
                config.TEAM = "red"

        if wpilib.SmartDashboard.getBoolean('DB/Button 2', self.button_2_last) != self.button_2_last:
            self.button_2_last = not self.button_2_last
            if config.AUTO == "five":
                config.AUTO = "two"
            else:
                config.AUTO = "five"
        # print(Sensors.color_sensors.get_val_left(), Sensors.color_sensors.get_val_right())
        # print(Robot.elevator.mag_sensor.get_value())
        # print(Robot.shooter.mag_sensor.get_value())
        # print(Robot.index.photo_electric.get_value())

        # logger.info(f"TURRET CURRENT POSITION IN DEGREES: {math.degrees(Robot.shooter.m_turret.get_sensor_position()/constants.turret_angle_gear_ratio)}")

    def teleopInit(self) -> None:
        Robot.elevator.initialized = False
        commands2.CommandScheduler.getInstance().schedule(DriveSwerveCustom(Robot.drivetrain))
        commands2.CommandScheduler.getInstance().schedule(BallPath(Robot.index))
        commands2.CommandScheduler.getInstance().schedule(TurretAim(Robot.shooter))
        commands2.CommandScheduler.getInstance().schedule(ElevatorRezero(Robot.elevator))
        Robot.index.ball_queue = 0

        # This will become a command soon
        # while not Robot.shooter.mag_sensor.get_value():
        #     Robot.shooter.m_turret.set_raw_output(-.1)
        #     # Manually set raw output
        #
        # Robot.shooter.m_turret.set_raw_output(0)
        # Robot.shooter.m_turret.set_sensor_position(0)
        #
        # degrees = 125
        # radia = math.radians(degrees)
        #
        # print("TURRET DESIRED ANGLE: ", Robot.shooter.set_turret_angle(radia * rad))

    def teleopPeriodic(self) -> None:
        # print(Robot.drivetrain.gyro.get_robot_heading())
        print("Turret current angle: ", math.degrees(Robot.shooter.get_turret_rotation_angle()))

        # Robot.shooter.m_turret.set_target_velocity(4)
        pass

    def autonomousInit(self) -> None:
        self.auto_routine.run()

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def _simulationInit(self) -> None:
        ...

    def _simulationPeriodic(self) -> None:
        ...


if __name__ == "__main__":
    wpilib.run(_Robot)
