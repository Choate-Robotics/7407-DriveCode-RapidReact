from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup, TalonConfig
import wpilib
from robotpy_toolkit_7407.unum import Unum
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit, talon_sensor_vel_unit, talon_sensor_accel_unit

import constants
from sensors import LimitSwitch
from utils.can_optimizations import optimize_leader_talon, optimize_normal_talon, optimize_leader_talon_no_sensor, \
    optimize_normal_talon_no_sensor

_MOTOR_CFG = TalonConfig(
    0.1, 0, 0, 1023 / 20937,
    motion_cruise_velocity=15000*talon_sensor_vel_unit, motion_acceleration=50000*talon_sensor_accel_unit,
    neutral_brake=True
)


class Elevator(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(17, inverted=True), TalonFX(18, inverted=False), config=_MOTOR_CFG)
    solenoid: wpilib.DoubleSolenoid
    l_elevator = [LimitSwitch(2), LimitSwitch(3)]
    l_hanger_top = [LimitSwitch(4), LimitSwitch(5)]
    l_hanger_bottom = [LimitSwitch(6), LimitSwitch(7)]
    zeroed: bool

    def init(self):
        self.motors.init()
        optimize_leader_talon(self.motors.motors[0])
        optimize_normal_talon_no_sensor(self.motors.motors[1])
        self.solenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 4, 5)
        self.retract_solenoid()
        self.zeroed = self.bottomed_out()

    def set_height(self, h: Unum):
        self.motors.set_target_position(h * constants.elevator_gear_ratio)

    def get_height(self):
        return self.motors.get_sensor_position() / constants.elevator_gear_ratio

    def extend_solenoid(self):
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

    def retract_solenoid(self):
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

    def bar_on_climb_hooks(self) -> bool:
        return all(m.get_value() for m in self.l_elevator)

    def bar_on_grab_hooks(self) -> bool:
        return (self.l_hanger_top[0].get_value() or self.l_hanger_bottom[0].get_value()) and \
               (self.l_hanger_top[1].get_value() or self.l_hanger_bottom[1].get_value())

    def bottomed_out(self):
        return not(self.l_elevator[0].get_value() or self.l_elevator[1].get_value())
