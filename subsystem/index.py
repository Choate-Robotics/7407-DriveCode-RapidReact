from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonConfig

from sensors import LimitSwitch
from utils.can_optimizations import optimize_normal_talon_no_sensor

_MOTOR_CFG = TalonConfig(neutral_brake=True)


class Index(Subsystem):
    motor: TalonFX = TalonFX(16, inverted=False, config=_MOTOR_CFG)
    photo_electric = LimitSwitch(0)

    def init(self):
        self.motor.init()
        optimize_normal_talon_no_sensor(self.motor)
        self.ball_queue = 0
        self.running = False
        self.refresh = False
        

    def set(self, motor_speed: float):
        # TODO Velocity control
        self.motor.set_raw_output(motor_speed)
        if motor_speed > 0:
            self.running = True
        else:
            self.running = False
