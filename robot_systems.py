from ctre import SensorTimeBase
from sensors import limit_switch
import subsystem
import sensors
import wpilib

from sensors.color_sensors import ColorSensors


class Robot:
    drivetrain = subsystem.Drivetrain()
    intake = subsystem.Intake()
    index = subsystem.Index()
    elevator = subsystem.Elevator()
    shooter = subsystem.Shooter()

    limelight = sensors.Limelight()

    limit_switches = [
        sensors.LimitSwitch(0),
        sensors.LimitSwitch(1), # SHOOTER LEFT
        sensors.LimitSwitch(8),
        sensors.LimitSwitch(9)
    ]


class Pneumatics:
    compressor = wpilib.Compressor(1, wpilib.PneumaticsModuleType.REVPH)

    @classmethod
    def get_compressor(cls):
        return cls.compressor.enabled(), cls.compressor.getCurrent()


class Sensors:
    color_sensors: ColorSensors
