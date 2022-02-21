import utils

import wpilib
import commands2
from ctre import ControlMode
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.network.network_system import Network
#from robotpy_toolkit_7407.subsystem_templates.drivetrain import DriveSwerve
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import deg

#from command.drivetrain import DriveSwerveCustom
from oi.OI import OI
from robot_systems import Robot, Pneumatics
import time


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

        # OI
        OI.init()
        OI.map_controls()

        # Pneumatics
        #Pneumatics.compressor.enableAnalog(90, 120)

        logger.info("initialization complete")

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        self.network_counter -= 1
        if self.network_counter == 0:
            self.network_counter = self.loops_per_net_update
            Network.robot_send_status()

    def teleopInit(self) -> None:
        #Robot.shooter.target(10)
        pass

    def teleopPeriodic(self) -> None:
        #commands2.CommandScheduler.getInstance().schedule(DriveSwerveCustom(Robot.drivetrain))
        print(Pneumatics.get_compressor())
        pass

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def _simulationInit(self) -> None: ...
    def _simulationPeriodic(self) -> None: ...


if __name__ == "__main__":
    wpilib.run(_Robot, period=0.05)
