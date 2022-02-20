from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup, TalonConfig


_MOTOR_CFG = TalonConfig(neutral_brake=True)


class Index(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(15, inverted=True), TalonFX(16, inverted=False), config=_MOTOR_CFG)

    def init(self):
        self.motors.init()

    def set(self, motor_speed: float):
        # TODO Velocity control
        self.motors.set_raw_output(motor_speed)
