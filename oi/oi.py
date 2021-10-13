from typing import Tuple, List
import wpilib
import commands2 as commands
import commands2.button
import utils.logger as logger

import command.shooter
import command.turret
import command.intake
import command.hopper
import command.shifter
import command.index.index_motor
import oi.keymap as keymap
from oi.joysticks import Joysticks
from oi.keymap import Keymap
import subsystem


class OI:
    _intake_run_command: command.intake.IntakeRun
    _intake_run_reverse_command: command.intake.IntakeRunReverse

    @staticmethod
    def init() -> None:
        logger.info("initializing operator interface", "[oi]")

        Joysticks.joysticks = [
            wpilib.Joystick(keymap.DRIVER_CONTROLLER_ID),
            wpilib.Joystick(keymap.OPERATOR_CONTROLLER_ID)
        ]

        logger.info("initialization complete", "[oi]")

    @staticmethod
    def map_controls():
        logger.info("mapping controller buttons", "[oi]")

        # SHOOTER
        logger.info("mapping shooter buttons", "[oi.shooter]")
        OI._get_button(Keymap.Shooter.LOW_RETRACTED)\
            .whenPressed(command.shooter.ShooterLowRetracted())
        OI._get_button(Keymap.Shooter.HIGH_RETRACTED)\
            .whenPressed(command.shooter.ShooterHighRetracted())
        OI._get_button(Keymap.Shooter.LOW_EXTENDED)\
            .whenPressed(command.shooter.ShooterLowExtended())
        OI._get_button(Keymap.Shooter.HIGH_EXTENDED)\
            .whenPressed(command.shooter.ShooterHighExtended())
        OI._get_button(Keymap.Shooter.AIM)\
            .whileHeld(command.shooter.ShooterEnable())\
            .whileHeld(command.turret.TurretAim())
        OI._get_button(Keymap.Shooter.SHOOT)\
            .whileHeld(command.index.index_motor.IndexShoot())

        # INTAKE
        logger.info("mapping intake buttons", "[oi.intake]")
        OI._intake_run_command = command.intake.IntakeRun()
        OI._intake_run_reverse_command = command.intake.IntakeRunReverse()
        OI._get_button(Keymap.Intake.UP)\
            .whenPressed(command.intake.IntakeUp())
        OI._get_button(Keymap.Intake.DOWN)\
            .whenPressed(command.intake.IntakeDown())
        OI._get_button(Keymap.Intake.START)\
            .whenPressed(OI._intake_run_command)
        OI._get_button(Keymap.Intake.STOP)\
            .cancelWhenPressed(OI._intake_run_command)\
            .cancelWhenPressed(OI._intake_run_reverse_command)

        # HOPPER
        logger.info("mapping hopper buttons", "[oi.hopper]")
        OI._get_button(Keymap.Hopper.RUN)\
            .whileHeld(command.hopper.HopperRun())
        OI._get_button(Keymap.Hopper.RUN_REVERSE)\
            .whileHeld(command.hopper.HopperRunReverse())

        # SHIFTER
        logger.info("mapping shifter buttons", "[oi.shifter]")
        OI._get_button(Keymap.Shifter.HIGH)\
            .whenPressed(command.shifter.ShifterHighGear())
        OI._get_button(Keymap.Shifter.LOW)\
            .whenPressed(command.shifter.ShifterLowGear())

        # INDEX
        logger.info("mapping index buttons", "[oi.index]")
        OI._get_button(Keymap.Index.UP)\
            .whenPressed(command.index.IndexUp())
        OI._get_button(Keymap.Index.DOWN)\
            .whenPressed(command.index.IndexDown())

        logger.info("mapping complete", "[oi]")

    @staticmethod
    def _get_button(key: Tuple[int, int]) -> commands.button.Button:
        joystick_number = key[1]
        button_number = key[0]

        if button_number == keymap.controller.LT:
            return commands.button.Button(lambda: Joysticks.joysticks[joystick_number].getRawAxis(2) > 0.8)

        if button_number == keymap.controller.RT:
            return commands.button.Button(lambda: Joysticks.joysticks[joystick_number].getRawAxis(3) > 0.8)

        return commands.button.JoystickButton(Joysticks.joysticks[joystick_number], button_number)

