import logging
import time

import commands2
import wpilib
from commands2 import WaitCommand
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit

import config
from oi.keymap import Controllers
from oi.keymap import Keymap
from robot_systems import Robot
from robot_systems import Sensors
from subsystem import Index


class BallPath(SubsystemCommand[Index]):
    def __init__(self, subsystem):
        super().__init__(subsystem)

    def initialize(self) -> None:
        pass

    def execute(self) -> None:

        Sensors.color_sensors.multiplexer.writeBulk(bytes([0b0100]))
        left_color = Sensors.color_sensors.color()
        left_val = Sensors.color_sensors.get_val()
        Sensors.color_sensors.multiplexer.writeBulk(bytes([0b1000]))
        right_color = Sensors.color_sensors.color()
        right_val = Sensors.color_sensors.get_val()

        if left_val[0] != 0 and right_val[0] != 0:
            Sensors.color_sensors.working = True
        else:
            Sensors.color_sensors.working = False

        if Robot.intake.left_intake_down or Robot.intake.right_intake_down:
            Robot.intake.dinglebobs_in()
        else:
            Robot.intake.dinglebobs_off()

        # logging.info(f"Color Sensors {left_val, right_val}")

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        pass
