import math

import wpilib
import commands2
from ctre import ControlMode
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.network.network_system import Network
from robotpy_toolkit_7407.subsystem_templates.drivetrain import DriveSwerve
from robotpy_toolkit_7407.utils import logger

from oi.OI import OI
from robot_systems import Robot


class _Robot(wpilib.TimedRobot):
    """
    Main robot class. Initializes OI and subsystems, and runs the commands scheduler.
    """
    loops_per_net_update: int = 10
    network_counter: int

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

        for s in subsystems:
            s.init()

        OI.init()
        OI.map_controls()

        logger.info("initialization complete")

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        self.network_counter -= 1
        if self.network_counter == 0:
            self.network_counter = self.loops_per_net_update
            Network.robot_send_status()

    def teleopInit(self) -> None:
        commands2.CommandScheduler.getInstance().schedule(DriveSwerve(Robot.drivetrain))

    def teleopPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        Robot.drivetrain.n_00.set_angle_raw(0)
        Robot.drivetrain.n_01.set_angle_raw(0)
        Robot.drivetrain.n_10.set_angle_raw(0)
        Robot.drivetrain.n_11.set_angle_raw(0)

    x = 0

    def autonomousPeriodic(self) -> None:
        Robot.drivetrain.n_00.set_angle_raw(self.x)
        Robot.drivetrain.n_01.set_angle_raw(self.x)
        Robot.drivetrain.n_10.set_angle_raw(self.x)
        Robot.drivetrain.n_11.set_angle_raw(self.x)
        self.x += 0.005
        logger.info(f"x={self.x}")
        logger.info(f"sensor_pos={Robot.drivetrain.n_00.get_current_angle_raw()}")

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def _simulationInit(self) -> None: ...
    def _simulationPeriodic(self) -> None: ...


if __name__ == "__main__":
    wpilib.run(_Robot)
