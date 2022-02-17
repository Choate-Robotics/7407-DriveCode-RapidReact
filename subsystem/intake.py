from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup
import wpilib
import rev


class Intake(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(13, inverted=True), TalonFX(14, inverted=False))
    pneumatic_hub = wpilib.PneumaticHub(22)
    solenoidL = pneumatic_hub.makeSolenoid(0)
    solenoidR = pneumatic_hub.makeSolenoid(1)
    #motors: TalonGroup = TalonGroup(TalonFX(14, inverted=False))
    current_speed = 0

    def init(self):
        self.motors.init()

    def set(self, motor_speed: float):
        self.motors.set_raw_output(motor_speed)
    def down(self):
        self.solenoidL.set(True)
        self.solenoidR.set(True)
