from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup
import wpilib


class Elevator(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(17, inverted=True), TalonFX(18, inverted=False)) # TODO: Test inversion
    ElevatorSolenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 4, 5)
    speed = .2

    def init(self):
        self.motors.init()

    def up(self):
        self.motors.set_raw_output(self.speed)
    def down(self):
        self.motors.set_raw_output(-self.speed)
    def stop(self):
        self.motors.set_raw_output(0)

    def toggleElevator(self):
        if self.ElevatorSolenoid.get() == wpilib.DoubleSolenoid.Value.kOff:
            self.ElevatorSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.ElevatorSolenoid.toggle()
