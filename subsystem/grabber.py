import wpilib

from lib.subsystem import Subsystem


class Grabber(Subsystem):
    move_solenoid: wpilib.DoubleSolenoid
    grab_solenoid: wpilib.DoubleSolenoid

    def init(self):
        self.move_solenoid = wpilib.DoubleSolenoid(2, 3)
        self.grab_solenoid = wpilib.DoubleSolenoid(6, 7)

    def set_move(self, closed: bool = True):
        self.move_solenoid.set(wpilib.DoubleSolenoid.Value.kForward if closed else wpilib.DoubleSolenoid.Value.kReverse)

    def set_grab(self, on: bool = True):
        self.grab_solenoid.set(wpilib.DoubleSolenoid.Value.kForward if on else wpilib.DoubleSolenoid.Value.kReverse)

