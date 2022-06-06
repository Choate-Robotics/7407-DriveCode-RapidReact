# HARTFORD READY

import commands2
import wpilib
from commands2 import WaitCommand
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.network.network_system import Network
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import m, s
from wpimath.geometry import Pose2d

import config
from autonomous.auto_routine import AutoRoutine
from command import ElevatorRezero
import constants
from autonomous import two_ball_auto, five_ball_auto, five_ball_auto_blue, five_ball_auto_red
#from autonomous import five_ball_auto, two_ball_auto, three_ball_auto # TODO: Fix this
from command import BallPath
from command.drivetrain import DriveSwerveCustom
from oi.OI import OI
from robot_systems import Robot, Pneumatics, Sensors
import sensors
from sensors.color_sensors import ColorSensors
from sensors.field_odometry import FieldOdometry
from sensors.intake_cameras import IntakeCameras
from sensors.rev_digit import RevDigit
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

        # self.test_command = ShooterDataCollectCommand(Robot.shooter).alongWith(command.IndexOn)

    def robotInit(self):
        wpilib.LiveWindow.disableAllTelemetry() # Disable Telemetry on Robot Startup to reduce loop time
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

        Robot.intake_cameras = IntakeCameras(Robot.intake)  # TODO

        Robot.odometry = FieldOdometry(Robot.drivetrain)

        # if config.AUTO == "five":
        #     if config.TEAM == "red":
        #         self.auto_routine = five_ball_auto_red.routine
        #         wpilib.SmartDashboard.putString('DB/String 7', f'AUTO: RED')
        #     else:
        #         self.auto_routine = five_ball_auto_blue.routine
        #         wpilib.SmartDashboard.putString('DB/String 7', f'AUTO: BLUE')
            
        # else:
        #     self.auto_routine = two_ball_auto.routine
        #     wpilib.SmartDashboard.putString('DB/String 7', f'AUTO: TWO')
        #self.auto_routine = two_ball_auto.routine
        #wpilib.SmartDashboard.putString('DB/String 7', f'AUTO: TWO')
        
        self.auto_routine = five_ball_auto_red.routine
        self.auto_combo = "Five RED"
        self.emergency = False

        self.initial_pose = self.auto_routine.initial_robot_pose
        # self.initial_pose = Pose2d(0, 0, 0)

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
        wpilib.SmartDashboard.putString('DB/String 1', f'CALL SID?: {self.emergency}')
        wpilib.SmartDashboard.putString('DB/String 2', f'Compressor Value: {round(Pneumatics.compressor.getPressure(), 1)}')
        wpilib.SmartDashboard.putString('DB/String 3', f'D_Motor Temp (C): {(Robot.drivetrain.n_00.m_move._motor.getTemperature()+Robot.drivetrain.n_01.m_move._motor.getTemperature()+Robot.drivetrain.n_10.m_move._motor.getTemperature()+Robot.drivetrain.n_11.m_move._motor.getTemperature())/4}')
        #wpilib.SmartDashboard.putString('DB/String 5', f'Color Sensors: {"WORKING" if Sensors.color_sensors.working == True else "FAILED" if not Sensors.color_sensors.working else Sensors.color_sensors.working}')
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

    def teleopInit(self) -> None:
        Robot.elevator.initialized = False
        commands2.CommandScheduler.getInstance().schedule(DriveSwerveCustom(Robot.drivetrain))
        #commands2.CommandScheduler.getInstance().schedule(IndexAutoDrive(Robot.index))
        #commands2.CommandScheduler.getInstance().schedule(IntakeAutoEject(Robot.intake))
        commands2.CommandScheduler.getInstance().schedule(BallPath(Robot.index))
        commands2.CommandScheduler.getInstance().schedule(ElevatorRezero(Robot.elevator))
        Robot.index.ball_queue = 0
        # if not Robot.shooter.zeroed:
        #     commands2.CommandScheduler.getInstance().schedule(ShooterZero(Robot.shooter))
        # if not Robot.elevator.zeroed:
        #     commands2.CommandScheduler.getInstance().schedule(ElevatorZero(Robot.elevator))
        # commands2.CommandScheduler.getInstance().schedule(self.test_command)
        pass

    def teleopPeriodic(self) -> None:
        Robot.odometry.update()
        Robot.intake_cameras.read_camera_data()
        # SHOOTER MOTOR PID LOG __
        #print(f"M_TOP DIST: {Robot.shooter.m_top.get_sensor_velocity() - Robot.shooter.desired_m_top}, M_BOT DIST: {Robot.shooter.m_bottom.get_sensor_velocity() - Robot.shooter.desired_m_bottom}")

    def autonomousInit(self) -> None:
        self.auto_routine.run()

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def _simulationInit(self) -> None: ...
    def _simulationPeriodic(self) -> None: ...


if __name__ == "__main__":
    wpilib.run(_Robot)
