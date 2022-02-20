from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup, TalonConfig
from robotpy_toolkit_7407.utils.units import rad, m, s
from unum import Unum

import constants


class Shooter(Subsystem):
    # TODO Add PID constants
    m_top = TalonFX(21, inverted=False, config=TalonConfig(neutral_brake=False))
    m_bottom = TalonFX(19, inverted=True, config=TalonConfig(neutral_brake=False))
    m_angle = TalonFX(20, inverted=True, config=TalonConfig(neutral_brake=True))

    def init(self):
        self.m_top.init()
        self.m_bottom.init()
        self.m_angle.init()

    def set_angle(self, theta: Unum):
        self.m_angle.set_target_position(theta * constants.shooter_angle_gear_ratio)

    def set_flywheels(self, top_vel: Unum, bottom_vel: Unum):
        self.m_top.set_target_velocity(top_vel * constants.shooter_top_gear_ratio)
        self.m_bottom.set_target_velocity(bottom_vel * constants.shooter_bottom_gear_ratio)
