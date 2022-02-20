from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit, talon_sensor_vel_unit, talon_sensor_accel_unit
from robotpy_toolkit_7407.utils.units import rad, rev, inch, deg, m, mile, hour, s, ft

drivetrain_turn_gear_ratio = 3353.33 * talon_sensor_unit/rad
drivetrain_move_gear_ratio = (511 * inch) / (544318 * talon_sensor_unit)

track_width = 29 * inch  # TODO Verify

# TODO Maybe change these
drivetrain_max_vel = 20 * mile/hour
drivetrain_max_angular_vel = 2 * rev/s
