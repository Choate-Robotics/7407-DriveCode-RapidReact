from commands2 import InstantCommand

import utils

import wpilib
import commands2
from ctre import ControlMode
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.network.network_system import Network
from robotpy_toolkit_7407.subsystem_templates.drivetrain import DriveSwerve
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import deg, s, m

from command.drivetrain import DriveSwerveCustom
from command.elevator import ElevatorSetupCommand, ElevatorClimbCommand
from oi.OI import OI
from oi.keymap import Keymap
from robot_systems import Robot, Pneumatics, Sensors
import time

from sensors.color_sensors import ColorSensors


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

        # OI
        OI.init()
        OI.map_controls()

        # Pneumatics
        Pneumatics.compressor.enableAnalog(90, 120)

        Sensors.color_sensors = ColorSensors()

        logger.info("initialization complete")

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        self.network_counter -= 1
        if self.network_counter == 0:
            self.network_counter = self.loops_per_net_update
            Network.robot_send_status()

    def teleopInit(self) -> None:
        #commands2.CommandScheduler.getInstance().schedule(ElevatorSetupCommand)
        pass

    def teleopPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().schedule(DriveSwerveCustom(Robot.drivetrain))
        # print(Pneumatics.get_compressor())
        # for i in range(10):
        #     print(f"Limit Switch {i}: {Robot.limit_switches[i].get_value()}")
        # pass

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        # Robot.elevator.motors.set_raw_output(1)
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def _simulationInit(self) -> None: ...
    def _simulationPeriodic(self) -> None: ...


if __name__ == "__main__":
    wpilib.run(_Robot, period=0.05)
