# import iniconfig
import wpilib
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup, TalonConfig, ctre_motors
from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_vel_unit, talon_sensor_accel_unit
from robotpy_toolkit_7407.utils.units import meters

import constants
from sensors import LimitSwitch
from utils.can_optimizations import optimize_leader_talon, optimize_normal_talon_no_sensor

_MOTOR_CFG = TalonConfig(
    0.1, 0, 0, 1023 / 20937,
    motion_cruise_velocity=30000*ctre_motors.k_sensor_vel_to_rad_per_sec, motion_acceleration=50000*ctre_motors.k_sensor_accel_to_rad_per_sec_sq,
    neutral_brake=True
)


class Elevator(Subsystem):
    motors: TalonGroup = TalonGroup(TalonFX(17, inverted=True), TalonFX(18, inverted=False), config=_MOTOR_CFG)
    solenoid: wpilib.DoubleSolenoid
    l_elevator = [LimitSwitch(2), LimitSwitch(3)]
    l_hanger_top = [LimitSwitch(4), LimitSwitch(5)]
    l_hanger_bottom = [LimitSwitch(6), LimitSwitch(7)]
    mag_sensor = LimitSwitch(9)
    initialized: bool = False

    def init(self):
        self.motors.init()
        optimize_leader_talon(self.motors.motors[0])
        optimize_normal_talon_no_sensor(self.motors.motors[1])
        self.solenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 4, 5)
        self.retract_solenoid()

    def set_height(self, h: meters):
        self.motors.set_target_position(h * constants.elevator_gear_ratio)

    def get_height(self):
        return self.motors.get_sensor_position() / constants.elevator_gear_ratio

    def extend_solenoid(self):
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

    def retract_solenoid(self):
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

    def bar_on_climb_hooks(self) -> bool:
        return True
        # return all(m.get_value() for m in self.l_elevator)

    def bar_on_grab_hooks(self) -> bool:
        return (self.l_hanger_top[0].get_value() or self.l_hanger_bottom[0].get_value()) and \
               (self.l_hanger_top[1].get_value() or self.l_hanger_bottom[1].get_value())

    def bottomed_out(self):
        return not(self.l_elevator[0].get_value() or self.l_elevator[1].get_value())

    def zero_elevator(self):
        # slowly lower the elevator until it is in view of the mag sensor
        while(self.mag_sensor.get_value() == False):
            h = self.get_height()
            h -= 0.005
            self.set_height(h)

        # reset motor's sensor position to 0
        self.motors.set_target_position(0)
        self.motors.set_sensor_position(0)

    def set_climb_speed(self):
        motor_cfg = TalonConfig(motion_cruise_velocity=15000*ctre_motors.k_sensor_vel_to_rad_per_sec, motion_acceleration=50000*ctre_motors.k_sensor_accel_to_rad_per_sec_sq)
        
        for m in self.motors.motors:
            m._set_config(motor_cfg)

    def set_high_climb_speed(self):
        motor_cfg = TalonConfig(motion_cruise_velocity=30000*ctre_motors.k_sensor_vel_to_rad_per_sec, motion_acceleration=100000*ctre_motors.k_sensor_accel_to_rad_per_sec_sq)

        for m in self.motors.motors:
            m._set_config(motor_cfg)
