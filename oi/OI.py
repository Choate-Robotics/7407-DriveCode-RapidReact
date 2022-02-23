from commands2 import InstantCommand
from robotpy_toolkit_7407.utils import logger

from command.shooter import ShooterEnable
from oi.keymap import Keymap
from robot_systems import Robot
import command


class OI:
    @staticmethod
    def init() -> None:
        logger.info("initializing operator interface", "[oi]")

        logger.info("initialization complete", "[oi]")

    @staticmethod
    def map_controls():
        logger.info("mapping controller buttons", "[oi]")

        # Keymap.Drivetrain.RESET_GYRO().whenPressed(lambda: Robot.drivetrain.odometry.reset_angle())

        # def zero_motors():
        #     Robot.drivetrain.n_00.zero()
        #     Robot.drivetrain.n_01.zero()
        #     Robot.drivetrain.n_10.zero()
        #     Robot.drivetrain.n_11.zero()
        # Keymap.Drivetrain.REZERO_MOTORS().whenPressed(zero_motors)

        # Keymap.Elevator.ELEVATOR_UP().whileHeld(command.ElevatorUp)
        # Keymap.Elevator.ELEVATOR_UP().whenReleased(command.ElevatorStop)
        # Keymap.Elevator.ELEVATOR_DOWN().whileHeld(command.ElevatorDown)
        # Keymap.Elevator.ELEVATOR_DOWN().whenReleased(command.ElevatorStop)
        Keymap.Elevator.ELEVATOR_SOLENOID_TOGGLE().whenPressed(command.ElevatorSolenoidToggle)

        Keymap.Elevator.ELEVATOR_INIT().whenPressed(command.ElevatorSetupCommand)
        Keymap.Elevator.ELEVATOR_CLIMB().whenPressed(command.ElevatorClimbCommand)

        # Keymap.Intake.INTAKE_ON().whenPressed(command.IntakeOn)
        # Keymap.Intake.INTAKE_OFF().whenPressed(command.IntakeOff)
        # Keymap.Intake.INTAKE_LEFT_SOLENOID_TOGGLE().whenPressed(command.IntakeLeftSolenoidToggle)
        # Keymap.Intake.INTAKE_RIGHT_SOLENOID_TOGGLE().whenPressed(command.IntakeRightSolenoidToggle)

        Keymap.Index.INDEX_ON().whenPressed(command.IndexOn)
        Keymap.Index.INDEX_OFF().whenPressed(command.IndexOff)

        Keymap.Shooter.SHOOTER_ENABLE().whileHeld(ShooterEnable(Robot.shooter))

        logger.info("mapping complete", "[oi]")
