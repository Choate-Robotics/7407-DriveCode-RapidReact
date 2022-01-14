import command
import utils.logger as logger
from oi.keymap import Keymap
from robot_systems import Robot


class OI:
    @staticmethod
    def init() -> None:
        logger.info("initializing operator interface", "[oi]")

        logger.info("initialization complete", "[oi]")

    @staticmethod
    def map_controls():
        logger.info("mapping controller buttons", "[oi]")

        # Keymap.Grabber.UP().whenPressed(command.GrabberMoveControl(Robot.grabber, False))
        # Keymap.Grabber.DOWN().whenPressed(command.GrabberMoveControl(Robot.grabber, True))
        # Keymap.Grabber.OPEN().whenPressed(command.GrabberGrabControl(Robot.grabber, True))
        # Keymap.Grabber.CLOSE().whenPressed(command.GrabberGrabControl(Robot.grabber, False))

        logger.info("mapping complete", "[oi]")
