from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup, TalonConfig
from robotpy_toolkit_7407.utils.units import rad, m, s, deg
from robotpy_toolkit_7407.unum import Unum
import math

import constants
from utils.can_optimizations import optimize_normal_talon, optimize_leader_talon
from utils.shooter_targeting import ShooterTargeting


class Shooter(Subsystem):
    m_top = TalonFX(21, inverted=True, config=TalonConfig(
        0.09, 0.001, 7.5, 1023 / 20369, integral_zone=1000, max_integral_accumulator=100000,
        neutral_brake=False))
    m_bottom = TalonFX(19, inverted=False, config=TalonConfig(
        0.26, 0.002, 11.6, 1023 / 20101, integral_zone=1000, max_integral_accumulator=100000,
        neutral_brake=False))
    m_angle = TalonFX(20, inverted=True, config=TalonConfig(
        0.3, 0.005, 1, 1023 * 0.1 / 917, integral_zone=1000, max_integral_accumulator=10000,
        neutral_brake=True))

    sensor_zero_angle = 15 * deg

    def init(self):
        self.m_top.init()
        self.m_bottom.init()
        self.m_angle.init()
        optimize_leader_talon(self.m_top)
        optimize_leader_talon(self.m_bottom)
        optimize_normal_talon(self.m_angle)

    def set_launch_angle(self, theta: Unum):
        theta = 90 * deg - theta - self.sensor_zero_angle
        self.m_angle.set_target_position(theta * constants.shooter_angle_gear_ratio)

    def set_flywheels(self, top_vel: Unum, bottom_vel: Unum):
        self.m_top.set_target_velocity(top_vel * constants.shooter_top_gear_ratio)
        self.m_bottom.set_target_velocity(bottom_vel * constants.shooter_bottom_gear_ratio)

    def target(self, limelight_dist):
        vx, vy = ShooterTargeting.stationary_aim(limelight_dist)
        final_velocity = (vx**2 + vy**2)**.5 * m/s
        final_angle = math.atan(vy / vx) * rad
        self.set_launch_angle(final_angle)
        self.set_flywheels(final_velocity, final_velocity)

    def stop(self):
        self.m_top.set_raw_output(0)
        self.m_bottom.set_raw_output(0)

