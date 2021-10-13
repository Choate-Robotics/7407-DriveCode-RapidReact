import wpilib
import utils.logger as logger
import commands2 as commands

from oi import OI
from robot_systems import Robot
from utils.paths import CURRENT_PATH
from utils.network import Network
import subsystem
import command.drivetrain
import command.shooter
import command.index.index_manual_speed
import oi


class _Robot(wpilib.TimedRobot):
    """
    Main robot class. Initializes OI and subsystems, and runs the command scheduler.
    """

    def robotInit(self):
        """
        Called on robot startup. Here the subsystems and oi are all initialized.
        """
        if self.isReal():
            logger.Color = logger.NoColor  # Disable log colors when on the robot

        logger.info("initializing robot")

        # Initialize NetworkTables for communication with dashboard and limelight
        Network.init()

        # Initialize OI (controller buttons are mapped later but this initialized joysticks)
        OI.init()

        # Initialize all subsystems (motors and solenoids are initialized here)
        Robot.drivetrain.init()
        Robot.shooter.init()
        Robot.turret.init()
        Robot.intake.init()
        Robot.hopper.init()
        Robot.shifter.init()
        Robot.index.init()

        # Set default command
        Robot.index.setDefaultCommand(command.index.index_manual_speed.IndexManualSpeedController())

        # Map the controls now that all subsystems are initialized
        OI.map_controls()

        logger.info("initialization complete")

    def robotPeriodic(self):
        # Run the command scheduler
        commands.CommandScheduler.getInstance().run()

    def teleopInit(self) -> None:
        commands.CommandScheduler.getInstance().schedule(command.drivetrain.DriveArcade())

    def teleopPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        commands.CommandScheduler.getInstance().schedule(
            command.drivetrain.follow_path.get_command(Robot.drivetrain, CURRENT_PATH)
        )

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        if not self.isReal():
            subsystem.SimDrivetrain.LEFT_VEL_METERS_PER_SECOND = 0
            subsystem.SimDrivetrain.RIGHT_VEL_METERS_PER_SECOND = 0

    def disabledPeriodic(self) -> None:
        pass

    def _simulationInit(self) -> None: ...
    def _simulationPeriodic(self) -> None: ...


if __name__ == "__main__":
    wpilib.run(_Robot)
