from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup


class Shooter(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(0, inverted=False), TalonFX(1, inverted=True))
    current_speed = 0
    increment = 0.0001

    def init(self):
        self.motors.init()

    def set(self, motor_speed: float):
        self.motors.set_raw_output(motor_speed)
