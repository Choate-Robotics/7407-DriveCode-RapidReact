from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit, talon_sensor_vel_unit, talon_sensor_accel_unit
from robotpy_toolkit_7407.unum.units import cm
from robotpy_toolkit_7407.utils.units import rad, rev, inch, deg, m, mile, hour, s, ft
import math

TEAM = "red"

period = 0.08

# --- DRIVETRAIN ---

drivetrain_turn_gear_ratio = 3353.33 * talon_sensor_unit/rad
drivetrain_move_gear_ratio = (544318 * talon_sensor_unit) / (511 * inch)

track_width = 24.2 * inch

# TODO Maybe change these
drivetrain_max_vel = 20 * mile/hour
drivetrain_max_angular_vel = 2 * rev/s
drivetrain_max_climb_vel = 2 * mile/hour


# --- SHOOTER ---

shooter_top_wheel_radius = 2 * inch
shooter_bottom_wheel_radius = 2 * inch

# For m/s (ball exit velocity) to rad/s (motor velocity)
shooter_top_gear_ratio = (1 * rad / shooter_top_wheel_radius) * (38 / 42)
shooter_bottom_gear_ratio = (1 * rad / shooter_bottom_wheel_radius) * (1 / 1)

# Shooter angle to motor angle
shooter_angle_gear_ratio = (266 / 10) * (36 / 12) * rad/rad

# Targeting constants
air_resistance_constant = 0.048187
height_difference = 1.85  # meters # In reality about 1.76m, but a slightly higher height makes the ball bounce off the back of the HUB
gravity = 9.80665  # m/s^2
shooter_delay = 0.21  # seconds
acceptable_error = 0.25  # meters
ideal_entry_angle = -math.pi / 3  # desired angle for the ball to enter the hub
minimum_shooter_angle = math.pi / 4  # radians
max_shooter_angle = math.radians(75)
max_shooter_velocity = 45  # m/s

# --- ELEVATOR ---

elevator_gear_ratio = 20 * rev / (5.501 * inch)

elevator_extended_height = 27.75 * inch
elevator_below_extended_height = 21.75 * inch
elevator_min_bar_contact_height = 23 * inch
elevator_pull_down_height = 0.5 * cm
elevator_latch_height = 2.5 * inch
elevator_fire_height = 5.5 * inch
