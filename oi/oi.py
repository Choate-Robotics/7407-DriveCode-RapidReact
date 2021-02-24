from typing import Tuple
import wpilib
import commands2 as commands
import commands2.button
import utils.logger as logger

import command.shooter
import oi.keymap as keymap
from oi.keymap import Keymap
import subsystem


class OI:
    def __init__(self) -> None:
        logger.info("initializing operator interface", "[oi]")

        self.joysticks = [
            wpilib.Joystick(keymap.DRIVER_CONTROLLER_ID),
            wpilib.Joystick(keymap.OPERATOR_CONTROLLER_ID)
        ]

        logger.info("initialization complete", "[oi]")

    def map_controls(self, shooter: subsystem.Shooter):
        logger.info("mapping controller buttons", "[oi]")

        self._get_button(Keymap.Shooter.LOW_RETRACTED).whenPressed(command.shooter.ShooterLowRetracted(shooter))
        self._get_button(Keymap.Shooter.HIGH_RETRACTED).whenPressed(command.shooter.ShooterHighRetracted(shooter))
        self._get_button(Keymap.Shooter.LOW_EXTENDED).whenPressed(command.shooter.ShooterLowExtended(shooter))
        self._get_button(Keymap.Shooter.HIGH_EXTENDED).whenPressed(command.shooter.ShooterHighExtended(shooter))
        self._get_button(Keymap.Shooter.AIM).whenPressed(command.shooter.ShooterEnable(shooter))

        logger.info("mapping complete", "[oi]")

    def _get_button(self, key: Tuple[int, int]) -> commands.button.Button:
        joystick_number = key[1]
        button_number = key[0]

        if button_number == keymap.controller.LT:
            return commands.button.Button(lambda: self.joysticks[joystick_number].getRawAxis(2) > 0.8)

        if button_number == keymap.controller.RT:
            return commands.button.Button(lambda: self.joysticks[joystick_number].getRawAxis(3) > 0.8)

        return commands.button.JoystickButton(self.joysticks[joystick_number], button_number)

