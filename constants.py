import math

from robotpy_toolkit_7407.motors.ctre_motors import talon_sensor_unit, talon_sensor_vel_unit, talon_sensor_accel_unit
from robotpy_toolkit_7407.unum.units import cm
from robotpy_toolkit_7407.utils.units import rad, rev, inch, deg, m, mile, hour, s, ft
from wpilib import DriverStation
import config

period = 0.03

# --- DRIVETRAIN ---

drivetrain_turn_gear_ratio = (3353.33 * talon_sensor_unit/rad).asNumber()
drivetrain_move_gear_ratio = ((544318 * talon_sensor_unit) / (511 * inch)).asNumber(rad/m)

track_width = (24.2 * inch).asNumber(m)

# TODO Maybe change these
drivetrain_max_vel = (5 * mile/hour).asNumber(m/s) #20
drivetrain_target_max_vel = (7 * mile/hour).asNumber(m/s)
drivetrain_max_angular_vel = (2 * rev/s).asNumber(rad/s)
drivetrain_max_climb_vel = (2 * mile/hour).asNumber(m/s)


# --- SHOOTER ---

shooter_top_wheel_radius = (2 * inch).asNumber(m)
shooter_bottom_wheel_radius = (2 * inch).asNumber(m)

# For m/s (ball exit velocity) to rad/s (motor velocity)
shooter_top_gear_ratio = (1 / shooter_top_wheel_radius) * (38 / 42)
shooter_bottom_gear_ratio = (1 / shooter_bottom_wheel_radius) * (1 / 1)

# Shooter angle to motor angle
# shooter_angle_gear_ratio = (266 / 10) * (36 / 12)
shooter_angle_gear_ratio = 57.5

# TURRET RELATED THINGS:
turret_angle_gear_ratio = 50.347*(18/16) # How many motor radians in one rev of the turret
max_turret_power = .50
min_turret_power = .07  # Minimum power required to move turret
default_turret_power = .20  # Default power of turret
max_turret_positional_velocity = 50000  # setting for positional pid of turret; max turret velocity

# Targeting constants
air_resistance_constant = 0.048187
height_difference = 2.6  # meters # In reality about 1.76m, but a slightly higher height makes the ball bounce off the back of the HUB # 1.85
gravity = 9.80665  # m/s^2
shooter_delay = -0.1  # seconds
acceptable_error = 0.45 # meters twas .55
ideal_entry_angle = -math.pi / 5  # desired angle for the ball to enter the hub
minimum_shooter_angle = math.pi / 4  # radians
max_shooter_angle = math.radians(75)
max_shooter_velocity = 15  # m/s #30
limelight_horizontal_adjustment = .1 #.35

# --- INTAKE/INDEX ---

dual_intakes_down = False #When True, enables Both Intakes to be down. WARNING, WILL mess up Ballpath logic system, expect to use operator manual controls
default_intake_speed = 1 # as stated
intake_current_sensing = False # Enables dragging balls while not actually intaking when full
index_shooting_speed = .72 # shooting index speed
default_index_speed = .7 # as stated
index_intaking_speed = .7 # index speed when intaking ball
index_photo_electric_threshold = 2 # the threshold check for the intake ball gereration #4 is sweet spot
# --- ELEVATOR ---

elevator_gear_ratio = (20 * rev / (5.501 * inch)).asNumber(rad/m)

elevator_extended_height = (28 * inch).asNumber(m)
elevator_below_extended_height = (21.75 * inch).asNumber(m)
elevator_min_bar_contact_height = (23 * inch).asNumber(m)
elevator_pull_down_height = (0.5 * cm).asNumber(m)
elevator_latch_height = (2.5 * inch).asNumber(m)
elevator_fire_height = (5.5 * inch).asNumber(m)
elevator_more_below_extended_height = (17 * inch).asNumber(m)