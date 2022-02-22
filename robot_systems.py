from ctre import SensorTimeBase
from sensors import limit_switch
import subsystem
import sensors
import wpilib


class Robot:
    drivetrain = subsystem.Drivetrain()
    intake = subsystem.Intake()
    index = subsystem.Index()
    elevator = subsystem.Elevator()
    shooter = subsystem.Shooter()

    limelight = sensors.Limelight()

    limit_switches = [
        sensors.Limit_Switch(port = 0, reverse=False), # Photo, should be SHOOTER RIGHT
        sensors.Limit_Switch(port = 1), # SHOOTER LEFT
        sensors.Limit_Switch(port = 2), # ELEV RIGHT
        sensors.Limit_Switch(port = 3), # ELEV LEFT
        sensors.Limit_Switch(port = 4), # HANGER RIGHT TOP
        sensors.Limit_Switch(port = 5), # HANGER LEFT TOP
        sensors.Limit_Switch(port = 6), # HANGER RIGHT SIDE
        sensors.Limit_Switch(port = 7), # HANGER LEFT SIDE
        sensors.Limit_Switch(port = 8), # ELEV MAGN UP
        sensors.Limit_Switch(port = 9)  # ELEV MAGN DOWN
        ]

class Pneumatics:
    compressor = wpilib.Compressor(1, wpilib.PneumaticsModuleType.REVPH)
    # ADD PNEUMATIC HUB AND GET EVERYTHING FROM THAT

    def get_compressor():
        return Pneumatics.compressor.enabled(), Pneumatics.compressor.getCurrent()
