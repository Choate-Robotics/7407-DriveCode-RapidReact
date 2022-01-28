from robotpy_toolkit_7407.utils import logger

from oi.keymap import Keymap
from robot_systems import Robot
from subsystem.shooter import ShooterSettings


class OI:
    @staticmethod
    def init() -> None:
        logger.info("initializing operator interface", "[oi]")

        logger.info("initialization complete", "[oi]")

    @staticmethod
    def map_controls():
        logger.info("mapping controller buttons", "[oi]")

        def increment():
            Robot.shooter.increment += 0.0001
        Keymap.Shooter.INCREASE_INCREMENT().whenPressed(increment)

        def decrement():
            Robot.shooter.increment -= 0.0001
        Keymap.Shooter.DECREASE_INCREMENT().whenPressed(decrement)

        logger.info("mapping complete", "[oi]")
