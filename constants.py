from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit, talon_sensor_vel_unit, talon_sensor_accel_unit
from robotpy_toolkit_7407.utils.units import rad, rev, inch, deg, m, mile, hour, s, ft

# --- DRIVETRAIN ---

drivetrain_turn_gear_ratio = 3353.33 * talon_sensor_unit/rad
drivetrain_move_gear_ratio = (544318 * talon_sensor_unit) / (511 * inch)

track_width = 29 * inch  # TODO Verify

# TODO Maybe change these
drivetrain_max_vel = 20 * mile/hour
drivetrain_max_angular_vel = 2 * rev/s


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
height_difference = 2 #meters
gravity = 9.80665 # m/s^2
shooter_delay = 0.5 # seconds
acceptable_error = 0.25 #meters


# --- ELEVATOR ---

elevator_gear_ratio = 20 * rev / (4.084070449666731 * inch)

elevator_extended_height = 20 * inch
elevator_min_bar_contact_height = 16 * inch
elevator_latch_height = 5 * inch
elevator_swing_height = 10 * inch
