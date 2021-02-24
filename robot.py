import wpilib
import utils.logger as logger
import commands2 as commands

from utils.network import Network
import subsystem
import command.drivetrain
import command.shooter
import oi


class Robot(wpilib.TimedRobot):
    drivetrain: subsystem.Drivetrain
    shooter: subsystem.Shooter
    turret: subsystem.Turret
    intake: subsystem.Intake
    hopper: subsystem.Hopper
    shifter: subsystem.Shifter
    oi: oi.OI

    def robotInit(self):
        logger.info("initializing robot")

        Network.init()

        self.oi = oi.OI()

        self.drivetrain = subsystem.Drivetrain()
        self.drivetrain.setDefaultCommand(command.drivetrain.DriveArcade(self.drivetrain, self.oi))
        self.shooter = subsystem.Shooter()
        self.turret = subsystem.Turret()
        self.intake = subsystem.Intake()
        self.hopper = subsystem.Hopper()
        self.shifter = subsystem.Shifter()

        self.oi.map_controls(self.shooter, self.intake, self.hopper, self.shifter)

        logger.info("initialization complete")

    def robotPeriodic(self):
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


if __name__ == "__main__":
    wpilib.run(Robot)
