from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit, talon_sensor_vel_unit, talon_sensor_accel_unit
from robotpy_toolkit_7407.unum.units import cm
from robotpy_toolkit_7407.utils.units import rad, rev, inch, deg, m, mile, hour, s, ft
from wpilib import DriverStation
import CONFIG

TEAM = CONFIG.TEAM  # 'red', 'blue'

period = 0.08

# --- DRIVETRAIN ---

drivetrain_turn_gear_ratio = (3353.33 * talon_sensor_unit/rad).asNumber()
drivetrain_move_gear_ratio = ((544318 * talon_sensor_unit) / (511 * inch)).asNumber(rad/m)

track_width = (24.2 * inch).asNumber(m)

# TODO Maybe change these
drivetrain_max_vel = (20 * mile/hour).asNumber(m/s)
drivetrain_max_angular_vel = (2 * rev/s).asNumber(rad/s)
drivetrain_max_climb_vel = (2 * mile/hour).asNumber(m/s)


# --- SHOOTER ---

shooter_top_wheel_radius = (2 * inch).asNumber(m)
shooter_bottom_wheel_radius = (2 * inch).asNumber(m)

# For m/s (ball exit velocity) to rad/s (motor velocity)
shooter_top_gear_ratio = (1 / shooter_top_wheel_radius) * (38 / 42)
shooter_bottom_gear_ratio = (1 / shooter_bottom_wheel_radius) * (1 / 1)

# Shooter angle to motor angle
shooter_angle_gear_ratio = (266 / 10) * (36 / 12)

# Targeting constants
air_resistance_constant = 0.048187
height_difference = 1.85  # meters # In reality about 1.76m, but a slightly higher height makes the ball bounce off the back of the HUB
gravity = 9.80665  # m/s^2
shooter_delay = 0.5  # seconds
acceptable_error = 0.25  # meters


# --- ELEVATOR ---

elevator_gear_ratio = (20 * rev / (5.501 * inch)).asNumber(rad/m)

elevator_extended_height = (28 * inch).asNumber(m)
elevator_below_extended_height = (21.75 * inch).asNumber(m)
elevator_min_bar_contact_height = (23 * inch).asNumber(m)
elevator_pull_down_height = (0.5 * cm).asNumber(m)
elevator_latch_height = (2.5 * inch).asNumber(m)
elevator_fire_height = (5.5 * inch).asNumber(m)
