from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup
import time


class Elevator(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(17, inverted=True), TalonFX(18, inverted=False)) # TODO: Test inversion
    speed = .2

    def init(self):
        self.motors.init()

    def up(self):
        self.motors.set_raw_output(self.speed)
        time.sleep(.5)
        self.motors.set_raw_output(0)
    def down(self):
        self.motors.set_raw_output(-self.speed)
        time.sleep(.5)
        self.motors.set_raw_output(0)
    def stop(self):
        self.motors.set_raw_output(0)
