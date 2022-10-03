from commands2 import InstantCommand
from robotpy_toolkit_7407.utils import logger
from wpimath.geometry import Pose2d, Rotation2d

import command
import config
from command.drivetrain import DriveSwerveCustom, ShootWhileMoving
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

        def reset_gyro():
            Robot.drivetrain.gyro.reset_angle()
            Robot.drivetrain.odometry.resetPosition(Pose2d(0, 0, 0), Rotation2d(0))

        Keymap.Drivetrain.RESET_GYRO().whenPressed(reset_gyro)

        def zero_motors():
            Robot.drivetrain.n_00.zero()
            Robot.drivetrain.n_01.zero()
            Robot.drivetrain.n_10.zero()
            Robot.drivetrain.n_11.zero()

        def zero_ball_queue():
            Robot.shooter.ready = False
            if not Robot.index.photo_electric.get_value():
                Robot.index.ball_queue = 0
                Robot.index.refresh = True

        Keymap.Drivetrain.REZERO_MOTORS().whenPressed(zero_motors)

        # Keymap.Drivetrain.AIM_SWERVE() \
        #     .whileHeld(DriveSwerveAim(Robot.drivetrain)) \
        #     .whileHeld(command.ShooterEnable(Robot.shooter)) \
        #     .whenReleased(DriveSwerveCustom(Robot.drivetrain)) \
        #     .whenReleased(InstantCommand(zero_ball_queue))

        def stop_shooting():
            Robot.shooter.shooting_over = True

        def stage():
            #print("Stage Keybind")
            Robot.index.stage = True
            Robot.index.aiming = True
    
        def deStage():
            #print("DeStage Keybind")
            Robot.index.destageBall = True
            Robot.index.aiming = False
        def resetBall():
            #print("Reset ball keybind")
            Robot.index.resetBall = True

        Keymap.Drivetrain.AIM_SWERVE().whenPressed(stage).whenReleased(deStage)

        def driver_centric_enable():
            DriveSwerveCustom.driver_centric = True

        def driver_centric_disable():
            DriveSwerveCustom.driver_centric = False

        def driver_centric_reversed_enable():
            DriveSwerveCustom.driver_centric_reversed = True

        def driver_centric_reversed_disable():
            DriveSwerveCustom.driver_centric_reversed = False

        Keymap.Drivetrain.DRIVER_CENTRIC().whenPressed(driver_centric_enable).whenReleased(driver_centric_disable)
        Keymap.Drivetrain.DRIVER_CENTRIC_REVERSED().whenPressed(driver_centric_reversed_enable).whenReleased(
            driver_centric_reversed_disable)

        # Keymap.Elevator.ELEVATOR_UP().whileHeld(command.ElevatorUp)
        # Keymap.Elevator.ELEVATOR_UP().whenReleased(command.ElevatorStop)
        # Keymap.Elevator.ELEVATOR_DOWN().whileHeld(command.ElevatorDown)
        # Keymap.Elevator.ELEVATOR_DOWN().whenReleased(command.ElevatorStop)
        Keymap.Elevator.ELEVATOR_DOWN().whenPressed(command.ElevatorDown())

        Keymap.Elevator.ELEVATOR_INIT().whenPressed(command.ElevatorSetupCommand())
        Keymap.Elevator.ELEVATOR_CLIMB().whenPressed(command.ElevatorClimbCommand())

        def extend_dinglebob_runtime():
            Robot.index.dinglebob_run_extend = True
            # print("extended")

        def stop_dinglebob_runtime():
            Robot.index.dinglebob_run_extend = False
            # print("unextended")

        # Keymap.Intake.LEFT_INTAKE_TOGGLE() \
        #     .whenPressed(command.IntakeToggleLeft(Robot.intake)) \
        #     .whenReleased(command.IntakeToggleLeft(Robot.intake)) \
        Keymap.Intake.LEFT_INTAKE_TOGGLE() \
            .whileHeld(Robot.intake.toggle_left_intake) \
            .whenReleased(Robot.intake.left_intake_disable) \
            # .whenReleased( \
        #    InstantCommand(extend_dinglebob_runtime) \
        #    .andThen(WaitCommand(0.5).andThen(stop_dinglebob_runtime)))
        # TODO: CALL SID!!! - SID
        # Keymap.Intake.RIGHT_INTAKE_TOGGLE().whenPressed(command.IntakeToggleRight(Robot.intake)).whenReleased(command.IntakeToggleRight(Robot.intake)) ### TODO: SID SAYS CALL TO FIX INTAKES!!!!!

        Keymap.Intake.RIGHT_INTAKE_TOGGLE() \
            .whileHeld(Robot.intake.toggle_right_intake) \
            .whenReleased(Robot.intake.right_intake_disable)

        Keymap.Index.RESET_BALL().whenPressed(resetBall)
        def auto_intake_on():
            Robot.intake.AUTO_INTAKE = True

        def auto_intake_off():
            Robot.intake.AUTO_INTAKE = False

        # Keymap.Intake.AUTO_INTAKE_TOGGLE() \
        #     .whenPressed(auto_intake_on) \
        #     .whenReleased(auto_intake_off)

        def change_team_color():
            if config.TEAM == 'red':
                config.TEAM = 'blue'
            else:
                config.TEAM = 'red'

        def toggle_auto_eject():
            config.EJECT_ENABLE = not config.EJECT_ENABLE

        Keymap.BallPath.TOGGLE_AUTO_EJECT_COLOR().whenPressed(InstantCommand(change_team_color))
        Keymap.BallPath.TOGGLE_AUTO_EJECT().whenPressed(InstantCommand(toggle_auto_eject))  # .5

        def aim_on():
            if Robot.limelight.table.getNumber('tx', None) is not None \
                    and Robot.limelight.table.getNumber('tx', None) != 0:
                Robot.shooter.aiming = True
                command.DriveSwerveTurretAim(Robot.drivetrain)
                command.TurretDriveAim(Robot.shooter)

        def aim_off():
            Robot.shooter.aiming = False
            DriveSwerveCustom(Robot.drivetrain)
            command.TurretAim(Robot.shooter)

        Keymap.Shooter.SHOOTER_ENABLE().whenPressed(InstantCommand(aim_on)).whenReleased(InstantCommand(aim_off))
        # Keymap.Shooter.FENDER_SHOT().whileHeld(command.ShooterEnableAtDistance(Robot.shooter, .5))
        # Keymap.Shooter.SHOOTER_SHORT_EJECT().whileHeld(command.ShooterEnableAtDistance(Robot.shooter, .35))

        # Keymap.Shooter.SHOOTER_OFFSET_UP().whenPressed(command.ShooterOffsetUp())
        # Keymap.Shooter.SHOOTER_OFFSET_DOWN().whenPressed(command.ShooterOffsetDown())

        # Keymap.Shooter.SHOOTER_ENABLE()\
        #     .whileHeld(ShootWhileMoving(Robot.drivetrain, Robot.shooter))\
        #     .whenReleased(DriveSwerveCustom(Robot.drivetrain))

        logger.info("mapping complete", "[oi]")
