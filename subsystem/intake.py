from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup, TalonConfig
import wpilib

from utils.can_optimizations import optimize_normal_talon, optimize_leader_talon

_MOTOR_CFG = TalonConfig(neutral_brake=False)


class Intake(Subsystem):
    m_bottom: TalonGroup = TalonGroup(TalonFX(13, inverted=True), config=_MOTOR_CFG)
    # m_bottom: TalonGroup = TalonGroup(TalonFX(13, inverted=True), TalonFX(14, inverted=False), config=_MOTOR_CFG)
    m_top: TalonFX = TalonFX(15, inverted=True, config=_MOTOR_CFG)
    s_left: wpilib.DoubleSolenoid
    s_right: wpilib.DoubleSolenoid

    def init(self):
        self.m_bottom.init()
        self.m_top.init()
        self.s_left = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 0, 1)
        self.s_right = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 2, 3)
        optimize_leader_talon(self.m_bottom.motors[0])
        # optimize_normal_talon(self.m_bottom.motors[1])
        optimize_normal_talon(self.m_top)

    def set(self, bottom_speed: float, top_speed: float):
        # TODO Velocity control
        self.m_bottom.set_raw_output(bottom_speed)
        self.m_top.set_raw_output(top_speed)

    def toggle_left_intake(self):
        if self.s_left.get() == wpilib.DoubleSolenoid.Value.kOff:
            self.s_left.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.s_left.toggle()

    def toggle_right_intake(self):
        if self.s_right.get() == wpilib.DoubleSolenoid.Value.kOff:
            self.s_right.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.s_right.toggle()
