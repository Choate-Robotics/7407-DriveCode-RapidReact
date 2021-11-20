from lib.motor import Motor, PIDMotor
from lib.subsystem import Subsystem
from utils import logger


class DifferentialDrivetrain(Subsystem):
    m_left: PIDMotor = None
    m_right: PIDMotor = None

    def init(self):
        logger.info("initializing differential drivetrain", "[differential_drivetrain]")
        self.m_left.init()
        self.m_right.init()
        logger.info("initialization complete", "[differential_drivetrain]")

    def set_motor_percent_output(self, left: float, right: float):
        self.m_left.set_raw_output(left)
        self.m_right.set_raw_output(right)

    def set_motor_velocity(self, left: float, right: float):
        self.m_left.set_target_velocity(left)
        self.m_right.set_target_velocity(right)
