from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup


class Shooter(Subsystem):
    fly_wheels: TalonGroup = TalonGroup(TalonFX(19, inverted=True), TalonFX(21, inverted=False))
    angle_adjustment_motors: TalonGroup = TalonGroup(TalonFX(20, inverted=True))
    current_speed = 0

    def init(self):
        self.fly_wheels.init()
        self.angle_adjustment_motors.init()

    def setFlyWheels(self, motor_speed: float):
        self.fly_wheels.set_raw_output(motor_speed)
    def setAngle(self, motor_speed: float):
        self.angle_adjustment_motors.set_raw_output(motor_speed)
    
    
