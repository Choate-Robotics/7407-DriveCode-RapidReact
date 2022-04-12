# HARTFORD READY

import commands2
import wpilib
from commands2 import WaitCommand
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.network.network_system import Network
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import m, s

import CONFIG
from command import elevator_rezero
import constants
from autonomous import two_ball_auto, five_ball_auto_blue, five_ball_auto_red
#from autonomous import five_ball_auto, two_ball_auto, three_ball_auto # TODO: Fix this
from command import BallPath
from command.drivetrain import DriveSwerveCustom
from oi.OI import OI
from robot_systems import Robot, Pneumatics, Sensors
from sensors.color_sensors import ColorSensors
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
        self.auto_routines = [two_ball_auto.routine, five_ball_auto_red.routine]

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

        Robot.limelight.led_off()

        # Robot.intake_cameras = IntakeCameras() # TODO

        logger.info("initialization complete")
        #print(f"INITIALIZED TEAM COLOR: {constants.TEAM}")

    def robotPeriodic(self):
        # print(Robot.drivetrain.gyro._gyro.getRoll())
        print(f"TEAM COLOR: {Robot.TEAM}")
        Robot.rev_digit.update()
        commands2.CommandScheduler.getInstance().run()
        # Robot.intake_cameras.read_camera_data() # TODO
        Robot.limelight.update()

    def teleopInit(self) -> None:
        Robot.limelight.led_off()
        Robot.elevator.initialized = False
        commands2.CommandScheduler.getInstance().schedule(DriveSwerveCustom(Robot.drivetrain))
        #commands2.CommandScheduler.getInstance().schedule(IndexAutoDrive(Robot.index))
        #commands2.CommandScheduler.getInstance().schedule(IntakeAutoEject(Robot.intake))
        commands2.CommandScheduler.getInstance().schedule(BallPath(Robot.index))
        commands2.CommandScheduler.getInstance().schedule(elevator_rezero(Robot.elevator))
        Robot.index.ball_queue = 0
        # if not Robot.shooter.zeroed:
        #     commands2.CommandScheduler.getInstance().schedule(ShooterZero(Robot.shooter))
        # if not Robot.elevator.zeroed:
        #     commands2.CommandScheduler.getInstance().schedule(ElevatorZero(Robot.elevator))
        # commands2.CommandScheduler.getInstance().schedule(self.test_command)
        pass

    def teleopPeriodic(self) -> None:
        #print(Robot.TEAM)
        #print("QUEUE: ", Robot.index.ball_queue)
        # logger.info(Robot.drivetrain.odometry.getPose())
        # print(Pneumatics.get_compressor())
        #print(Robot.index.photo_electric.get_value())
        # for i in range(10):
        #     print(f"Limit Switch {i}: {Robot.limit_switches[i].get_value()}")
        # z = Robot.limelight.calculate_distance()
        # print(f"Limelight: {z}"
        # logger.info(f"{Robot.limelight.calculate_distance()}")
        #print("Sensor: ", Robot.index.motor.get_sensor_position())
        #print("BALL QUEUE: ", Robot.index.ball_queue)
        pass

    def autonomousInit(self) -> None:
        Robot.limelight.led_off()

        if CONFIG.AUTO == "five":
            if CONFIG.TEAM == "red":
                five_ball_auto_red.routine.run()
            else:
                five_ball_auto_blue.routine.run()
        else:
            two_ball_auto.routine.run()
        
        # self.auto_routines[Robot.rev_digit.routine_idx].run()
        # two_ball_auto.routine.run()
        # two_ball_auto.routine.run() # TODO: Fix this
        # Robot.elevator.set_height(0 * inch)
        # Robot.shooter.target(5)
        pass

    def autonomousPeriodic(self) -> None:
        # logger.info(Robot.drivetrain.odometry.getPose())
        # c = ""
        # for i, sw in enumerate(Robot.limit_switches):
        #     if sw.get_value():
        #         c += f"{i} "
        # if c != "":
        #     logger.info(c)
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def _simulationInit(self) -> None: ...
    def _simulationPeriodic(self) -> None: ...


if __name__ == "__main__":
    wpilib.run(_Robot)
