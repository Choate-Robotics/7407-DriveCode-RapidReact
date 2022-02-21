from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup, TalonConfig

from utils.can_optimizations import optimize_normal_talon, optimize_leader_talon

_MOTOR_CFG = TalonConfig(neutral_brake=True)


class Index(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(15, inverted=True), TalonFX(16, inverted=False), config=_MOTOR_CFG)

    def init(self):
        self.motors.init()
        optimize_leader_talon(self.motors.motors[0])
        optimize_normal_talon(self.motors.motors[1])

    def set(self, motor_speed: float):
        # TODO Velocity control
        self.motors.set_raw_output(motor_speed)
