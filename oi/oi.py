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
import command.hanger
import command.index.index_motor
import oi.keymap as keymap
from oi.buttons import AxisButton
from oi.joysticks import Joysticks
from oi.keymap import Keymap
import subsystem


class OI:
    _intake_run_command: command.intake.IntakeRun
    _intake_run_reverse_command: command.intake.IntakeRunReverse

    @staticmethod
    def init() -> None:
        logger.info("initializing operator interface", "[oi]")

        logger.info("initialization complete", "[oi]")

    @staticmethod
    def map_controls():
        logger.info("mapping controller buttons", "[oi]")

        # SHOOTER
        logger.info("mapping shooter buttons", "[oi.shooter]")
        Keymap.Shooter.LOW_RETRACTED()\
            .whenPressed(command.shooter.ShooterLowRetracted())
        Keymap.Shooter.HIGH_RETRACTED()\
            .whenPressed(command.shooter.ShooterHighRetracted())
        Keymap.Shooter.LOW_EXTENDED()\
            .whenPressed(command.shooter.ShooterLowExtended())
        Keymap.Shooter.HIGH_EXTENDED()\
            .whenPressed(command.shooter.ShooterHighExtended())
        Keymap.Shooter.AIM()\
            .whileHeld(command.shooter.ShooterEnable())\
            .whileHeld(command.turret.TurretAim())
        Keymap.Shooter.SHOOT()\
            .whileHeld(command.index.index_motor.IndexShoot())

        # INTAKE
        logger.info("mapping intake buttons", "[oi.intake]")
        OI._intake_run_command = command.intake.IntakeRun()
        OI._intake_run_reverse_command = command.intake.IntakeRunReverse()
        Keymap.Intake.UP()\
            .whenPressed(command.intake.IntakeUp())
        Keymap.Intake.DOWN()\
            .whenPressed(command.intake.IntakeDown())
        Keymap.Intake.START()\
            .whenPressed(OI._intake_run_command)
        Keymap.Intake.STOP()\
            .cancelWhenPressed(OI._intake_run_command)\
            .cancelWhenPressed(OI._intake_run_reverse_command)

        # HOPPER
        logger.info("mapping hopper buttons", "[oi.hopper]")
        Keymap.Hopper.RUN()\
            .whileHeld(command.hopper.HopperRun())
        Keymap.Hopper.RUN_REVERSE()\
            .whileHeld(command.hopper.HopperRunReverse())
        AxisButton(keymap.Controllers.OPERATOR, Keymap.Index.MANUAL_INDEX_BOTTOM_AXIS, range_max=-0.2)()\
            .whileHeld(command.hopper.HopperRunReverse())

        # SHIFTER
        logger.info("mapping shifter buttons", "[oi.shifter]")
        Keymap.Shifter.HIGH()\
            .whenPressed(command.shifter.ShifterHighGear())
        Keymap.Shifter.LOW()\
            .whenPressed(command.shifter.ShifterLowGear())

        # INDEX
        logger.info("mapping index buttons", "[oi.index]")
        Keymap.Index.UP()\
            .whenPressed(command.index.IndexUp())
        Keymap.Index.DOWN()\
            .whenPressed(command.index.IndexDown())

        # HANGER
        logger.info("mapping hanger buttons", "[oi.hanger]")
        Keymap.Hanger.HANGER_INIT()\
            .whenPressed(commands.ParallelDeadlineGroup(commands.WaitCommand(1), command.hanger.HangerControl(-0.3)))
        Keymap.Hanger.HANGER_CLIMB()\
            .whileHeld(command.hanger.HangerControl(-1))

        logger.info("mapping complete", "[oi]")
