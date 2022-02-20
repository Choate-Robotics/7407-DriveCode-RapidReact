from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup
import wpilib

from constants import optimize_talon


# TODO Motion magic
class Elevator(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(17, inverted=True), TalonFX(18, inverted=False))  # TODO: Test inversion
    solenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 4, 5)
    speed = .2

    def init(self):
        self.motors.init()
        optimize_talon(self.motors.motors[0]._motor)
        optimize_talon(self.motors.motors[1]._motor)

    def up(self):
        self.motors.set_raw_output(self.speed)

    def down(self):
        self.motors.set_raw_output(-self.speed)

    def stop(self):
        self.motors.set_raw_output(0)

    def toggle_solenoid(self):
        if self.solenoid.get() == wpilib.DoubleSolenoid.Value.kOff:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.solenoid.toggle()
