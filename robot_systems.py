from ctre import SensorTimeBase

import constants
from sensors import limit_switch
import subsystem
import sensors
import wpilib
import config

from sensors.color_sensors import ColorSensors
from sensors.field_odometry import FieldOdometry
from sensors.rev_digit import RevDigit
from sensors.intake_cameras import IntakeCameras
from sensors.limelight import Limelight


class Robot:
    drivetrain = subsystem.Drivetrain()
    intake = subsystem.Intake()
    index = subsystem.Index()
    elevator = subsystem.Elevator()
    shooter = subsystem.Shooter()

    limelight: Limelight
    odometry: FieldOdometry
    rev_digit: RevDigit
    intake_cameras: IntakeCameras


class Pneumatics:
    compressor = wpilib.Compressor(1, wpilib.PneumaticsModuleType.REVPH)

    @classmethod
    def get_compressor(cls):
        return cls.compressor.enabled(), cls.compressor.getCurrent()


class Sensors:
    color_sensors: ColorSensors
