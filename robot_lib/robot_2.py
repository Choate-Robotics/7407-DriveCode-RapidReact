from typing import List, Type

import wpilib
import utils.logger as logger
import commands2 as commands

from robot_lib import subsystem
from utils.paths import CURRENT_PATH
from utils.network import Network


class _Robot(wpilib.TimedRobot):
    subsystems: List[commands.SubsystemBase]

    def robotInit(self):
        if self.isReal():
            logger.Color = logger.NoColor  # Disable log colors when on the robot

        logger.info("initializing robot")

        Network.init()

        self.subsystems = [s() for s in subsystem._SUBSYSTEMS]

        logger.info("initialization complete")

    def robotPeriodic(self):
        # Run the command scheduler
        commands.CommandScheduler.getInstance().run()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
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


def run_robot():
    wpilib.run(_Robot)
