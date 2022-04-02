from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonGroup, TalonConfig
from robotpy_toolkit_7407.utils.units import rad, m, s, deg
from robotpy_toolkit_7407.unum import Unum
import math

import constants
from sensors import LimitSwitch
from utils.can_optimizations import optimize_normal_talon, optimize_leader_talon, optimize_normal_talon_no_sensor
from utils.shooter_targeting import ShooterTargeting


class Shooter(Subsystem):
    m_top = TalonFX(21, inverted=True, config=TalonConfig(
        0.09, 0, 5, 1023 / 20369, integral_zone=1000, max_integral_accumulator=100000,
        neutral_brake=False))
    m_bottom = TalonFX(19, inverted=False, config=TalonConfig(
        0.1, 0, 0.5, 1023 / 20101, integral_zone=1000, max_integral_accumulator=100000,
        neutral_brake=False))
    # OLD VALUES
    # m_top = TalonFX(21, inverted=True, config=TalonConfig(
    #     0.09, 0.001, 5, 1023 / 20369, integral_zone=1000, max_integral_accumulator=100000,
    #     neutral_brake=False))
    # m_bottom = TalonFX(19, inverted=False, config=TalonConfig(
    #     0.14, 0.002, 10, 1023 / 20101, integral_zone=1000, max_integral_accumulator=100000,
    #     neutral_brake=False))
    m_angle = TalonFX(20, inverted=True, config=TalonConfig(
        0.3, 0.005, 1, 1023 * 0.1 / 917, integral_zone=1000, max_integral_accumulator=10000,
        neutral_brake=True))

    sensor_zero_angle = 15 * deg
    angle_range = 45 * deg

    left_limit = LimitSwitch(1)
    zeroed: bool

    offset_m = -0.12

    def init(self):
        self.m_top.init()
        self.m_bottom.init()
        self.m_angle.init()
        optimize_normal_talon(self.m_top)
        optimize_normal_talon(self.m_bottom)
        optimize_normal_talon(self.m_angle)
        self.zeroed = self.left_limit.get_value()

        self.drive_ready = False
        self.shooter_ready = False

    def set_launch_angle(self, theta: Unum):
        theta = 90 * deg - theta - self.sensor_zero_angle
        self.m_angle.set_target_position(max(min(theta, self.angle_range), 0 * rad) * constants.shooter_angle_gear_ratio)

    def set_flywheels(self, top_vel: Unum, bottom_vel: Unum):
        self.m_top.set_target_velocity(top_vel * constants.shooter_top_gear_ratio)
        self.m_bottom.set_target_velocity(bottom_vel * constants.shooter_bottom_gear_ratio)
        tolerance = .1
        if abs(self.m_top.get_sensor_velocity()-top_vel*constants.shooter_top_gear_ratio)<tolerance*(top_vel*constants.shooter_top_gear_ratio) and abs(self.m_bottom.get_sensor_velocity()-bottom_vel*constants.shooter_bottom_gear_ratio)<tolerance*(bottom_vel*constants.shooter_bottom_gear_ratio): 
            self.shooter_ready = True
        else:
            self.shooter_ready = False

        print(self.shooter_ready, self.drive_ready)
    def set_flywheels_for_ball_velocity(self, vx: float, vy: float):

        final_velocity = (-0.286 + 1.475 * (vx**2 + vy**2)**.5) * m/s
        final_angle = math.atan(vy / vx) * rad
        self.set_launch_angle(final_angle)
        self.set_flywheels(final_velocity, final_velocity)

    def target_stationary(self, limelight_dist):
        vx, vy = ShooterTargeting.stationary_aim(limelight_dist)
        self.set_flywheels_for_ball_velocity(vx, vy)
   
    def target_with_motion(self, limelight_dist, angle_to_hub, robot_vel) -> float:
        adjusted_robot_vel = ShooterTargeting.real_velocity_to_shooting(robot_vel, angle_to_hub)
        (vx, vy), theta = ShooterTargeting.moving_aim_ahead(angle_to_hub, adjusted_robot_vel, limelight_dist)
        self.set_flywheels_for_ball_velocity(vx, vy)
        return theta

    def stop(self):
        self.m_top.set_raw_output(0)
        self.m_bottom.set_raw_output(0)
        self.m_angle.set_target_position(0 * rad)
