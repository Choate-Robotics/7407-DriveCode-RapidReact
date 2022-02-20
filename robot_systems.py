import subsystem
import wpilib


class Robot:
    drivetrain = subsystem.Drivetrain()
    intake = subsystem.Intake()
    index = subsystem.Index()
    elevator = subsystem.Elevator()
    shooter = subsystem.Shooter()


class Pneumatics:
    compressor = wpilib.Compressor(1, wpilib.PneumaticsModuleType.REVPH)
    # ADD PNEUMATIC HUB AND GET EVERYTHING FROM THAT

    def get_compressor(self):
        return self.compressor.enabled(), self.compressor.getCurrent()

