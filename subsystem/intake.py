from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup, TalonConfig
import wpilib

from utils.can_optimizations import optimize_normal_talon, optimize_leader_talon, optimize_leader_talon_no_sensor, \
    optimize_normal_talon_no_sensor

_MOTOR_CFG = TalonConfig(neutral_brake=False)


class Intake(Subsystem):
    m_bottom_l: TalonFX = TalonFX(14, inverted=False, config=_MOTOR_CFG)
    m_bottom_r: TalonFX = TalonFX(13, inverted=True, config=_MOTOR_CFG)
    m_top: TalonFX = TalonFX(15, inverted=True, config=_MOTOR_CFG)
    s_left: wpilib.DoubleSolenoid
    s_right: wpilib.DoubleSolenoid
    on_l: bool
    on_r: bool

    def init(self):
        self.m_bottom_l.init()
        self.m_bottom_r.init()
        self.m_top.init()
        self.s_left = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 0, 1)
        self.s_right = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 2, 3)
        optimize_normal_talon_no_sensor(self.m_bottom_l)
        optimize_normal_talon_no_sensor(self.m_bottom_r)
        optimize_normal_talon_no_sensor(self.m_top)
        self.on_l = False
        self.on_r = False

    def set_left(self, on: bool):
        if self.on_l != on:
            self.toggle_left()

    def set_right(self, on: bool):
        if self.on_r != on:
            self.toggle_right()

    def toggle_left(self):
        if self.on_l:
            self.m_bottom_l.set_raw_output(0)
            self.m_top.set_raw_output(.7 if self.on_r else 0)
            self.s_left.set(wpilib.DoubleSolenoid.Value.kReverse)
            self.on_l = False
        else:
            self.m_bottom_l.set_raw_output(.5)
            self.m_top.set_raw_output(.7)
            self.s_left.set(wpilib.DoubleSolenoid.Value.kForward)
            self.on_l = True

    def toggle_right(self):
        if self.on_r:
            self.m_bottom_r.set_raw_output(0)
            self.m_top.set_raw_output(.7 if self.on_l else 0)
            self.s_right.set(wpilib.DoubleSolenoid.Value.kReverse)
            self.on_r = False
        else:
            self.m_bottom_r.set_raw_output(.5)
            self.m_top.set_raw_output(.7)
            self.s_right.set(wpilib.DoubleSolenoid.Value.kForward)
            self.on_r = True
