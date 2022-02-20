from ctre import SensorTimeBase
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


class Pneumatics:
    compressor = wpilib.Compressor(1, wpilib.PneumaticsModuleType.REVPH)
    # ADD PNEUMATIC HUB AND GET EVERYTHING FROM THAT

    def get_compressor(self):
        return self.compressor.enabled(), self.compressor.getCurrent()

