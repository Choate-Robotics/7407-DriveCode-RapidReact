from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit, talon_sensor_vel_unit, talon_sensor_accel_unit
from robotpy_toolkit_7407.utils.units import rad, rev, inch, deg, m, mile, hour, s, ft

# --- DRIVETRAIN ---

drivetrain_turn_gear_ratio = 3353.33 * talon_sensor_unit/rad
drivetrain_move_gear_ratio = (511 * inch) / (544318 * talon_sensor_unit)

track_width = 29 * inch  # TODO Verify

# TODO Maybe change these
drivetrain_max_vel = 20 * mile/hour
drivetrain_max_angular_vel = 2 * rev/s


# --- SHOOTER ---

# For m/s (ball exit velocity) to rad/s (motor velocity)
shooter_top_gear_ratio = 1 * rad/m  # TODO Find this
shooter_bottom_gear_ratio = 1 * rad/m  # TODO Find this

# Shooter angle to motor angle
shooter_angle_gear_ratio = (266 / 10) * (36 / 12) * rad/rad  # TODO Find this

air_resistance_constant = 0.048187
height_difference = 6
gravity = 9.80665