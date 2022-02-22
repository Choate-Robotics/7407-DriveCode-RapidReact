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
        sensors.LimitSwitch(port=0, reverse=False),  # Photo, should be SHOOTER RIGHT
        sensors.LimitSwitch(port=1),  # SHOOTER LEFT
        sensors.LimitSwitch(port=2),  # ELEV RIGHT
        sensors.LimitSwitch(port=3),  # ELEV LEFT
        sensors.LimitSwitch(port=4),  # HANGER RIGHT TOP
        sensors.LimitSwitch(port=5),  # HANGER LEFT TOP
        sensors.LimitSwitch(port=6),  # HANGER RIGHT SIDE
        sensors.LimitSwitch(port=7),  # HANGER LEFT SIDE
        sensors.LimitSwitch(port=8),  # ELEV MAGN UP
        sensors.LimitSwitch(port=9)   # ELEV MAGN DOWN
    ]


class Pneumatics:
    compressor = wpilib.Compressor(1, wpilib.PneumaticsModuleType.REVPH)

    @classmethod
    def get_compressor(cls):
        return cls.compressor.enabled(), cls.compressor.getCurrent()


class Sensors:
    color_sensors: ColorSensors
