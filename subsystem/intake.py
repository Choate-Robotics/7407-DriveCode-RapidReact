from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonSRX


class Intake(Subsystem):
    m_left: TalonSRX = TalonSRX(10, True)
    m_right: TalonSRX = TalonSRX(11, False)

    def init(self):
        self.m_left.init()
        self.m_right.init()

    def set(self, speed):
        self.m_left.set_raw_output(speed)
        self.m_right.set_raw_output(speed)
