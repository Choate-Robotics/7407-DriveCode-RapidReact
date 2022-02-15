from robotpy_toolkit_7407.utils import logger

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

        #Keymap.Drivetrain.RESET_GYRO().whenPressed(lambda: Robot.drivetrain.odometry.reset_angle())

        # def zero_motors():
        #     Robot.drivetrain.n_00.zero()
        #     Robot.drivetrain.n_01.zero()
        #     Robot.drivetrain.n_10.zero()
        #     Robot.drivetrain.n_11.zero()
        # Keymap.Drivetrain.REZERO_MOTORS().whenPressed(zero_motors)

        logger.info("mapping complete", "[oi]")

        Keymap.Elevator.ELEVATOR_UP().whileHeld(command.ElevatorUp(Robot.elevator))
        Keymap.Elevator.ELEVATOR_DOWN().whileHeld(command.ElevatorDown(Robot.elevator))
