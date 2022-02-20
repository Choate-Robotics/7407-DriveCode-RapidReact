from robotpy_toolkit_7407 import Subsystem
import wpilib


class Pneumatics(Subsystem):
    pneumatic_hub = wpilib.PneumaticHub(1)
    LeftIntakeSolenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 0, 1)
    RightIntakeSolenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 2, 3)
    ElevatorSolenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 4, 5)
    compressor = wpilib.Compressor(1, wpilib.PneumaticsModuleType.REVPH)
    current_speed = 0

    def init(self):
        self.compressor.enableAnalog(90, 120)

    def toggle_left_intake(self):
        if self.LeftIntakeSolenoid.get() == wpilib.DoubleSolenoid.Value.kOff:
            self.LeftIntakeSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.LeftIntakeSolenoid.toggle()

    def toggle_right_intake(self):
        if self.RightIntakeSolenoid.get() == wpilib.DoubleSolenoid.Value.kOff:
            self.RightIntakeSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.RightIntakeSolenoid.toggle()

    def toggle_elevator(self):
        if self.ElevatorSolenoid.get() == wpilib.DoubleSolenoid.Value.kOff:
            self.ElevatorSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.ElevatorSolenoid.toggle()

    def get_compressor(self):
        return self.compressor.enabled(), self.compressor.getCurrent()
