from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup
import wpilib


class Intake(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(13, inverted=True), TalonFX(14, inverted=False))
    LeftIntakeSolenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 0, 1)
    RightIntakeSolenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 2, 3)
    #motors: TalonGroup = TalonGroup(TalonFX(14, inverted=False))

    def init(self):
        self.motors.init()

    def set(self, motor_speed: float):
        self.motors.set_raw_output(motor_speed)

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
