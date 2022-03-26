from commands2 import InstantCommand
from robotpy_toolkit_7407.utils import logger
from wpimath.geometry import Pose2d, Rotation2d

from command.drivetrain import DriveSwerveCustom
from command.shoot_while_moving import ShootWhileMoving
from oi.keymap import Keymap
from robot_systems import Robot
import command
from command import DriveSwerveAim


class OI:
    @staticmethod
    def init() -> None:
        logger.info("initializing operator interface", "[oi]")

        logger.info("initialization complete", "[oi]")

    @staticmethod
    def map_controls():
        logger.info("mapping controller buttons", "[oi]")

        def reset_gyro():
            Robot.drivetrain.gyro.reset_angle()
            Robot.drivetrain.odometry.resetPosition(Pose2d(0, 0, 0), Rotation2d(0))

        Keymap.Drivetrain.RESET_GYRO().whenPressed(reset_gyro)

        def zero_motors():
            Robot.drivetrain.n_00.zero()
            Robot.drivetrain.n_01.zero()
            Robot.drivetrain.n_10.zero()
            Robot.drivetrain.n_11.zero()

        Keymap.Drivetrain.REZERO_MOTORS().whenPressed(zero_motors)

        Keymap.Drivetrain.AIM_SWERVE() \
            .whileHeld(DriveSwerveAim(Robot.drivetrain)) \
            .whenReleased(DriveSwerveCustom(Robot.drivetrain))

        # Keymap.Elevator.ELEVATOR_UP().whileHeld(command.ElevatorUp)
        # Keymap.Elevator.ELEVATOR_UP().whenReleased(command.ElevatorStop)
        # Keymap.Elevator.ELEVATOR_DOWN().whileHeld(command.ElevatorDown)
        # Keymap.Elevator.ELEVATOR_DOWN().whenReleased(command.ElevatorStop)
        Keymap.Elevator.ELEVATOR_SOLENOID_TOGGLE().whenPressed(command.ElevatorSolenoidToggle())

        Keymap.Elevator.ELEVATOR_INIT().whenPressed(command.ElevatorSetupCommand())
        Keymap.Elevator.ELEVATOR_CLIMB().whenPressed(command.ElevatorClimbCommand())

        Keymap.Intake.LEFT_INTAKE_TOGGLE().whenPressed(command.IntakeToggleLeft())
        Keymap.Intake.RIGHT_INTAKE_TOGGLE().whenPressed(command.IntakeToggleRight())
        Keymap.Intake.INTAKE_REVERSE_TOGGLE().whenPressed(command.IntakeToggleReverse())
        Keymap.Shooter.SHOOTER_ENABLE().whileHeld(command.ShooterEnable(Robot.shooter))
        Keymap.Shooter.SHOOTER_EJECT().whileHeld(command.ShooterEnableAtDistance(Robot.shooter, .5))

        # Keymap.Shooter.SHOOTER_ENABLE()\
        #     .whileHeld(ShootWhileMoving(Robot.drivetrain, Robot.shooter))\
        #     .whenReleased(DriveSwerveCustom(Robot.drivetrain))

        logger.info("mapping complete", "[oi]")
