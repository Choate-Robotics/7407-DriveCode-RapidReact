from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup


class Index(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(15, inverted=True), TalonFX(16, inverted=False))
    current_speed = 0

    def init(self):
        self.motors.init()

    def set(self, motor_speed: float):
        self.motors.set_raw_output(motor_speed)
