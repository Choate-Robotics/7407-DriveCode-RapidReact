import wpilib
from robotpy_toolkit_7407 import Subsystem


class Grabber(Subsystem):
    move_solenoid: wpilib.DoubleSolenoid
    grab_solenoid: wpilib.DoubleSolenoid

    def init(self):
        self.move_solenoid = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 2, 3)
        self.grab_solenoid = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 6, 7)

    def set_move(self, closed: bool = True):
        self.move_solenoid.set(wpilib.DoubleSolenoid.Value.kForward if not closed else wpilib.DoubleSolenoid.Value.kReverse)

    def set_grab(self, on: bool = True):
        self.grab_solenoid.set(wpilib.DoubleSolenoid.Value.kForward if not on else wpilib.DoubleSolenoid.Value.kReverse)

