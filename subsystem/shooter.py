import math

import ctre
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import TalonFX, TalonConfig, ctre_motors
from robotpy_toolkit_7407.utils.units import rad, deg, radians, meters_per_second, m, s

import constants
from sensors import LimitSwitch
from utils.can_optimizations import optimize_normal_talon
from utils.shooter_targeting import ShooterTargeting

from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit


class Shooter(Subsystem):
    m_top = TalonFX(8, inverted=False, config=TalonConfig(
        0.09, 0, 5 * 1.25, 1023 / 20369, integral_zone=1000, max_integral_accumulator=100000,
        neutral_brake=False))
    m_bottom = TalonFX(5, inverted=False, config=TalonConfig(
        0.1, 0, 0.5 * 1.25, 1023 / 20101, integral_zone=1000, max_integral_accumulator=100000,
        neutral_brake=False))
    m_angle = TalonFX(9, inverted=False, config=TalonConfig(
        0.3, 0.005, 2, 2046 * 0.1 / 917, integral_zone=1000, max_integral_accumulator=10000,  # 1023 * 0.1 / 917
        neutral_brake=True, motion_cruise_velocity=6000 * ctre_motors.k_sensor_vel_to_rad_per_sec,
        motion_acceleration=10000 * ctre_motors.k_sensor_accel_to_rad_per_sec_sq))

    max_turret_accel = constants.max_turret_positional_velocity  # 100000 50000 "20000 Works. 20% max works" "40000 Works. 40% max works"`

    m_turret = TalonFX(20, inverted=False, config=TalonConfig(
        k_P=.2, k_I=0, k_D=0, k_F=1023 / 20101, integral_zone=10000, max_integral_accumulator=100000,
        neutral_brake=True, motion_cruise_velocity=12000 * ctre_motors.k_sensor_vel_to_rad_per_sec,
        motion_acceleration=max_turret_accel * ctre_motors.k_sensor_accel_to_rad_per_sec_sq))

    # BEST ACCELERATION FOR TURRET Position PID IS 100000

    sensor_zero_angle = (15 * deg).asNumber(rad)
    turret_zero_units = 0
    turret_max_units = 70459 * talon_sensor_unit
    turret_max_angle = 246
    angle_range = (45 * deg).asNumber(rad)
    turret_range = (236 * deg).asNumber(rad)

    max_turret_turn_velocity = 1 * m / s

    left_limit = LimitSwitch(1)

    mag_sensor = LimitSwitch(8)

    zeroed: bool
    ready: bool

    offset_m = -0.12

    prev_flywheel_vel = (0, 0)

    shooting_over: bool = False

    desired_m_top = 0
    desired_m_bottom = 0

    turret_zeroed: bool

    target_turret_dist = None # in meeters
    target_turret_angle = None # in degrees

    seen_after_drivetrain_rezero: bool

    def __init__(self):
        super().__init__()

    def init(self):
        self.m_top.init()
        self.m_bottom.init()
        self.m_angle.init()
        self.m_turret.init()
        optimize_normal_talon(self.m_top)
        optimize_normal_talon(self.m_bottom)
        optimize_normal_talon(self.m_angle)
        optimize_normal_talon(self.m_turret)
        self.zeroed = self.left_limit.get_value()
        self.ready = False
        self.shooting_over = False
        self.aiming = False

        self.desired_turret_angle = None

        self.turret_zeroed = self.mag_sensor.get_value()

        if self.turret_zeroed:
            self.m_turret.set_sensor_position(0)

        self.seen_after_drivetrain_rezero = False
        # self.target_turret_dist = None
        # self.target_turget_angle = None # (IN RADIANS)

        self.auto_finished = True

    def set_launch_angle(self, theta: radians):
        theta = math.radians(90) - theta - self.sensor_zero_angle
        print("TARGET ANGLE: ", theta * constants.shooter_angle_gear_ratio)
        self.m_angle.set_target_position(max(min(theta, self.angle_range), 0) * constants.shooter_angle_gear_ratio)

    def set_turret_angle(self, theta: radians):
        self.m_turret.set_target_position(theta * constants.turret_angle_gear_ratio)

    def get_turret_rotation_velocity(self):
        return self.m_turret.get_sensor_velocity() / constants.turret_angle_gear_ratio

    def get_turret_rotation_angle(self):
        return self.m_turret.get_sensor_position() / constants.turret_angle_gear_ratio

    def set_turret_rotation_velocity(self, vel: meters_per_second):
        if abs(vel) > self.max_turret_turn_velocity:
            self.m_turret.set_target_velocity(self.max_turret_turn_velocity * constants.turret_angle_gear_ratio)
        else:
            self.m_turret.set_target_velocity(vel * constants.turret_angle_gear_ratio)

    def set_flywheels(self, top_vel: meters_per_second, bottom_vel: meters_per_second):
        self.desired_m_top = top_vel * constants.shooter_top_gear_ratio
        self.desired_m_bottom = bottom_vel * constants.shooter_bottom_gear_ratio

        self.m_top.set_target_velocity(self.desired_m_top)
        self.m_bottom.set_target_velocity(self.desired_m_bottom)
        # print("M_TOP_DESIRED: ", self.desired_m_top)
        # print("M_BOTTOM_DESIRED: ", self.desired_m_bottom)

    def set_flywheels_for_ball_velocity(self, vx: meters_per_second, vy: meters_per_second):
        self.prev_flywheel_vel = (vx, vy)
        final_velocity = (-0.286 + 1.475 * (vx ** 2 + vy ** 2) ** .5)
        final_angle = math.atan(vy / vx)
        # print("FINAL ANGLE: ", final_angle)
        self.set_launch_angle(final_angle)
        self.set_flywheels(final_velocity, final_velocity)

    def get_current_ball_exit_velocity(self) -> tuple[meters_per_second, meters_per_second]:
        v1 = self.m_top.get_sensor_velocity() / constants.shooter_top_gear_ratio
        v2 = self.m_bottom.get_sensor_velocity() / constants.shooter_bottom_gear_ratio
        v = 0.5 * (v1 + v2)
        v_adj = (v + 0.286) / 1.475
        sensor_theta = self.m_angle.get_sensor_position() / constants.shooter_angle_gear_ratio
        launch_angle = math.radians(90) - sensor_theta - self.sensor_zero_angle
        return v_adj * math.cos(launch_angle), v_adj * math.sin(launch_angle)

    def target_stationary(self, limelight_dist):
        limelight_dist += constants.limelight_horizontal_adjustment
        vx, vy = ShooterTargeting.stationary_aim(limelight_dist)
        self.set_flywheels_for_ball_velocity(vx, vy)

    def target_with_motion(self, limelight_dist, angle_to_hub, robot_vel) -> tuple[float, bool]:
        limelight_dist -= 0.3
        adjusted_robot_vel = ShooterTargeting.real_velocity_to_shooting(robot_vel, angle_to_hub)
        if setting := ShooterTargeting.moving_aim_ahead(angle_to_hub, adjusted_robot_vel, limelight_dist):
            if setting[0] is None or setting[1] is None:
                self.set_flywheels_for_ball_velocity(*self.prev_flywheel_vel)
                return angle_to_hub, False
            (vx, vy), theta = setting
            self.set_flywheels_for_ball_velocity(vx, vy)
            v_current = self.get_current_ball_exit_velocity()
            try:
                should_shoot = ShooterTargeting.should_shoot(
                    angle_to_hub,
                    adjusted_robot_vel,
                    limelight_dist,
                    v_current
                )
            except ValueError:
                should_shoot = False
            return -theta, should_shoot
        else:
            self.set_flywheels_for_ball_velocity(*self.prev_flywheel_vel)
            return angle_to_hub, False

    def stop(self):
        self.m_top.set_raw_output(0)
        self.m_bottom.set_raw_output(0)
        self.m_angle.set_target_position(0)
