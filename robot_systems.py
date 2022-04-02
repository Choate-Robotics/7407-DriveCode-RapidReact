from ctre import SensorTimeBase
from sensors import limit_switch
import subsystem
import sensors
import wpilib

from sensors.color_sensors import ColorSensors
from sensors.rev_digit import RevDigit


class Robot:
    drivetrain = subsystem.Drivetrain()
    intake = subsystem.Intake()
    index = subsystem.Index()
    elevator = subsystem.Elevator()
    shooter = subsystem.Shooter()

    limelight = sensors.Limelight()

    rev_digit: RevDigit



class Pneumatics:
    compressor = wpilib.Compressor(1, wpilib.PneumaticsModuleType.REVPH)

    @classmethod
    def get_compressor(cls):
        return cls.compressor.enabled(), cls.compressor.getCurrent()


class Sensors:
    color_sensors: ColorSensors
