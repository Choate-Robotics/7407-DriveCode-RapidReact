from commands2 import InstantCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain.swerve_drivetrain_commands import FollowPath
from wpimath.geometry import Pose2d, Rotation2d

import command
import constants
from autonomous.follow_path import FollowPathCustom
from autonomous.trajectory import generate_trajectory, TrajectoryEndpoint, generate_trajectory_from_pose
from command import IndexDrive
from sensors import limelight, Limelight
from subsystem.shooter import Shooter

import utils

import wpilib
import commands2
from ctre import ControlMode
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.network.network_system import Network
from robotpy_toolkit_7407.subsystem_templates.drivetrain import DriveSwerve
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import deg, s, m, inch, rad

from command.drivetrain import DriveSwerveCustom
from command.elevator import ElevatorZero, ElevatorSetupCommand, ElevatorClimbCommand
from command.shooter import ShooterZero
from oi.OI import OI
from oi.keymap import Keymap
from robot_systems import Robot, Pneumatics, Sensors
import time

from sensors.color_sensors import ColorSensors
from utils.shooter_data import ShooterDataCollectCommand


class _Robot(wpilib.TimedRobot):
    """
    Main robot class. Initializes OI and subsystems, and runs the command scheduler.
    """
    loops_per_net_update: int = 10
    network_counter: int

    def __init__(self):
        super().__init__(constants.period)

        # self.test_command = ShooterDataCollectCommand(Robot.shooter).alongWith(command.IndexOn)

    def robotInit(self):
        """
        Called on robot startup. Here the subsystems and oi are all initialized.
        """
        if self.isReal():
            logger.Color = logger.NoColor  # Disable log colors when on the robot

        logger.info("initializing robot")

        subsystems: list[Subsystem] = list(
            {k: v for k, v in Robot.__dict__.items() if isinstance(v, Subsystem)}.values()
        )

        Network.robot_init(subsystems)
        self.network_counter = self.loops_per_net_update

        for subsystem in subsystems:
            subsystem.init()

        # OI
        OI.init()
        OI.map_controls()

        # Pneumatics
        Pneumatics.compressor.enableAnalog(90, 120)

        Sensors.color_sensors = ColorSensors()

        commands2.CommandScheduler.getInstance().setPeriod(constants.period)

        Robot.limelight.led_on()

        logger.info("initialization complete")

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        Robot.limelight.update()
        self.network_counter -= 1
        if self.network_counter == 0:
            self.network_counter = self.loops_per_net_update
            Network.robot_send_status()

    def teleopInit(self) -> None:
        commands2.CommandScheduler.getInstance().schedule(DriveSwerveCustom(Robot.drivetrain))
        commands2.CommandScheduler.getInstance().schedule(IndexDrive(Robot.index))
        Robot.limelight.led_on()
        # if not Robot.shooter.zeroed:
        #     commands2.CommandScheduler.getInstance().schedule(ShooterZero(Robot.shooter))
        # if not Robot.elevator.zeroed:
        #     commands2.CommandScheduler.getInstance().schedule(ElevatorZero(Robot.elevator))
        # commands2.CommandScheduler.getInstance().schedule(self.test_command)
        pass

    def teleopPeriodic(self) -> None:
        logger.info(Robot.drivetrain.odometry.getPose())
        # print(Pneumatics.get_compressor())
        # for i in range(10):
        #     print(f"Limit Switch {i}: {Robot.limit_switches[i].get_value()}")
        # z = Robot.limelight.calculate_distance()
        # print(f"Limelight: {z}")
        # logger.info(f"{Robot.limelight.calculate_distance()}")
        pass

    def autonomousInit(self) -> None:
        Robot.limelight.led_on()
        Robot.drivetrain.odometry.resetPosition(Pose2d(0, 0, 0), Rotation2d(0))
        commands2.CommandScheduler.getInstance().schedule(
            FollowPathCustom(
                Robot.drivetrain,
                generate_trajectory_from_pose(
                    Pose2d(0, 0, (90 * deg).asNumber(rad)), [],
                    TrajectoryEndpoint(0 * m, 2 * m), 4 * m/s, 2 * m/(s*s)
                ),
                90 * deg,
                constants.period
            )
        )
        # Robot.elevator.set_height(0 * inch)
        # Robot.shooter.target(5)
        pass

    def autonomousPeriodic(self) -> None:
        logger.info(Robot.drivetrain.odometry.getPose())
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
