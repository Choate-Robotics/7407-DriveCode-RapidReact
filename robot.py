import wpilib
import utils.logger as logger
import commands2 as commands

from utils.network import Network
import subsystem
import command.drivetrain
import command.shooter
import command.index.index_manual_speed
import oi


class Robot(wpilib.TimedRobot):
    """
    Main robot class. Initializes OI and subsystems, and runs the command scheduler.
    """
    drivetrain: subsystem.Drivetrain
    shooter: subsystem.Shooter
    turret: subsystem.Turret
    intake: subsystem.Intake
    hopper: subsystem.Hopper
    shifter: subsystem.Shifter
    index: subsystem.Index
    oi: oi.OI

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
        self.oi = oi.OI()

        # Initialize all subsystems (motors and solenoids are initialized here)
        if self.isReal():
            self.drivetrain = subsystem.Drivetrain()
        else:
            self.drivetrain = subsystem.SimDrivetrain()
        self.shooter = subsystem.Shooter()
        self.turret = subsystem.Turret()
        self.intake = subsystem.Intake()
        self.hopper = subsystem.Hopper()
        self.shifter = subsystem.Shifter()
        self.index = subsystem.Index()

        # Set default command
        self.index.setDefaultCommand(command.index.index_manual_speed.IndexManualSpeedController(self.index, self.oi))

        # Map the controls now that all subsystems are initialized
        self.oi.map_controls(self.shooter, self.turret, self.intake, self.hopper, self.shifter, self.index)

        logger.info("initialization complete")

    def robotPeriodic(self):
        # Run the command scheduler
        commands.CommandScheduler.getInstance().run()

    def teleopInit(self) -> None:
        commands.CommandScheduler.getInstance().schedule(command.drivetrain.DriveArcade(self.drivetrain, self.oi))

    def teleopPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        commands.CommandScheduler.getInstance().schedule(command.drivetrain.FollowPath(self.drivetrain))

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
    wpilib.run(Robot)
