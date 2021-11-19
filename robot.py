import wpilib
import utils.logger as logger
import commands2 as commands

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
        Robot.hanger.init()

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
        drive_forward = command.drivetrain.DriveStraight(5000).as_timed(1)
        aim = commands.ParallelDeadlineGroup(commands.WaitCommand(5), command.turret.TurretAim(), command.shooter.ShooterEnable())
        shoot = commands.ParallelDeadlineGroup(commands.WaitCommand(5), command.index.IndexShoot(), command.shooter.ShooterEnable())
        cmd = commands.SequentialCommandGroup(command.shooter.ShooterHighExtended(), aim, shoot, drive_forward)
        commands.CommandScheduler.getInstance().schedule(cmd)

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
