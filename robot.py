import wpilib
import utils.logger as logger
import commands2

from lib.subsystem import Subsystem
from lib.subsystem_templates.drivetrain.swerve_drivetrain_commands import DriveSwerve
from oi.OI import OI
from robot_systems import Robot
from lib.network.network_system import Network


class _Robot(wpilib.TimedRobot):
    """
    Main robot class. Initializes OI and subsystems, and runs the commands scheduler.
    """
    loops_per_net_update: int = 10
    network_counter: int

    def robotInit(self):
        """
        Called on robot startup. Here the subsystems and oi are all initialized.
        """
        if self.isReal():
            logger.Color = logger.NoColor  # Disable log colors when on the robot

        logger.info("initializing robot")

        subsystems: list[Subsystem] = list(
            {k: v for k, v in Robot.__dict__.items() if isinstance(v, Subsystem)}.values()
        )

        Network.robot_init(subsystems)
        self.network_counter = self.loops_per_net_update

        for s in subsystems:
            s.init()

        OI.init()
        OI.map_controls()

        logger.info("initialization complete")

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        self.network_counter -= 1
        if self.network_counter == 0:
            self.network_counter = self.loops_per_net_update
            Network.robot_send_status()

    def teleopInit(self) -> None:
        commands2.CommandScheduler.getInstance().schedule(DriveSwerve(Robot.drivetrain))

    def teleopPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        Robot.drivetrain.n_00.m_rotate.set_target_position(0)
        Robot.drivetrain.n_01.m_rotate.set_target_position(0)
        Robot.drivetrain.n_10.m_rotate.set_target_position(0)
        Robot.drivetrain.n_11.m_rotate.set_target_position(0)

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def _simulationInit(self) -> None: ...
    def _simulationPeriodic(self) -> None: ...


if __name__ == "__main__":
    wpilib.run(_Robot)
