from typing import Tuple
import wpilib
import commands2 as commands
import commands2.button
import utils.logger as logger

import command.shooter
import command.turret
import command.intake
import command.hopper
import command.shifter
import oi.keymap as keymap
from oi.keymap import Keymap
import subsystem


class OI:
    _intake_run_command: command.intake.IntakeRun
    _intake_run_reverse_command: command.intake.IntakeRunReverse

    def __init__(self) -> None:
        logger.info("initializing operator interface", "[oi]")

        self.joysticks = [
            wpilib.Joystick(keymap.DRIVER_CONTROLLER_ID),
            wpilib.Joystick(keymap.OPERATOR_CONTROLLER_ID)
        ]

        logger.info("initialization complete", "[oi]")

    def map_controls(self, shooter: subsystem.Shooter, turret: subsystem.Turret, intake: subsystem.Intake, hopper: subsystem.Hopper,
                     shifter: subsystem.Shifter):
        logger.info("mapping controller buttons", "[oi]")

        # SHOOTER
        logger.info("mapping shooter buttons", "[oi.shooter]")
        self._get_button(Keymap.Shooter.LOW_RETRACTED)\
            .whenPressed(command.shooter.ShooterLowRetracted(shooter))
        self._get_button(Keymap.Shooter.HIGH_RETRACTED)\
            .whenPressed(command.shooter.ShooterHighRetracted(shooter))
        self._get_button(Keymap.Shooter.LOW_EXTENDED)\
            .whenPressed(command.shooter.ShooterLowExtended(shooter))
        self._get_button(Keymap.Shooter.HIGH_EXTENDED)\
            .whenPressed(command.shooter.ShooterHighExtended(shooter))
        self._get_button(Keymap.Shooter.AIM)\
            .whenPressed(command.shooter.ShooterEnable(shooter))\
            .whenPressed(command.turret.TurretAim(turret))

        # INTAKE
        logger.info("mapping intake buttons", "[oi.intake]")
        self._intake_run_command = command.intake.IntakeRun(intake)
        self._intake_run_reverse_command = command.intake.IntakeRunReverse(intake)
        self._get_button(Keymap.Intake.UP)\
            .whenPressed(command.intake.IntakeUp(intake))
        self._get_button(Keymap.Intake.DOWN)\
            .whenPressed(command.intake.IntakeDown(intake))
        self._get_button(Keymap.Intake.START)\
            .whenPressed(self._intake_run_command)
        self._get_button(Keymap.Intake.STOP)\
            .cancelWhenPressed(self._intake_run_command)\
            .cancelWhenPressed(self._intake_run_reverse_command)

        # HOPPER
        logger.info("mapping hopper buttons", "[oi.hopper]")
        self._get_button(Keymap.Hopper.RUN)\
            .whenPressed(command.hopper.HopperRun(hopper))
        self._get_button(Keymap.Hopper.RUN_REVERSE)\
            .whenPressed(command.hopper.HopperRunReverse(hopper))

        # SHIFTER
        logger.info("mapping shifter buttons", "[oi.shifter]")
        self._get_button(Keymap.Shifter.HIGH).\
            whenPressed(command.shifter.ShifterHighGear(shifter))
        self._get_button(Keymap.Shifter.LOW).\
            whenPressed(command.shifter.ShifterLowGear(shifter))

        logger.info("mapping complete", "[oi]")

    def _get_button(self, key: Tuple[int, int]) -> commands.button.Button:
        joystick_number = key[1]
        button_number = key[0]

        if button_number == keymap.controller.LT:
            return commands.button.Button(lambda: self.joysticks[joystick_number].getRawAxis(2) > 0.8)

        if button_number == keymap.controller.RT:
            return commands.button.Button(lambda: self.joysticks[joystick_number].getRawAxis(3) > 0.8)

        return commands.button.JoystickButton(self.joysticks[joystick_number], button_number)

