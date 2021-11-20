import wpilib
import utils.logger as logger
import commands2

from lib.subsystem import Subsystem
from lib.subsystem_templates.drivetrain.differential_drivetrain_commands import DriveArcade
from robot_systems import Robot
from utils.network import Network


class _Robot(wpilib.TimedRobot):
    """
    Main robot class. Initializes OI and subsystems, and runs the commands scheduler.
    """

    def robotInit(self):
        """
        Called on robot startup. Here the subsystems and oi are all initialized.
        """
        if self.isReal():
            logger.Color = logger.NoColor  # Disable log colors when on the robot

        logger.info("initializing robot")

        Network.init()

        subsystems: list[Subsystem] = list({k: v for k, v in Robot.__dict__.items() if isinstance(v, Subsystem)}.values())

        for s in subsystems:
            s.init()

        logger.info("initialization complete")

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    def teleopInit(self) -> None:
        commands2.CommandScheduler.getInstance().schedule(DriveArcade(Robot.drivetrain))

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


if __name__ == "__main__":
    wpilib.run(_Robot)
