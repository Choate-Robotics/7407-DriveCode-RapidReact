from robotpy_toolkit_7407.utils import logger


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
        #
        # Keymap.Intake.IN().whileHeld(command.IntakeRunCommand(Robot.intake, 0.3))
        # Keymap.Intake.OUT().whileHeld(command.IntakeRunCommand(Robot.intake, -0.3))

        logger.info("mapping complete", "[oi]")
