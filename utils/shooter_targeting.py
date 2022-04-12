import math
import numpy

from constants import air_resistance_constant, height_difference, gravity, acceptable_error, shooter_delay, \
    ideal_entry_angle, minimum_shooter_angle, max_shooter_angle, max_shooter_velocity


class ShooterTargeting:
    @staticmethod
    def time_up(velocity_up):
        time = math.atan(velocity_up * math.sqrt(air_resistance_constant / gravity)) / (
            math.sqrt(gravity * air_resistance_constant))
        return time

    @staticmethod
    def distance_up(time, velocity_up):
        distance = (1 / air_resistance_constant) * math.log(math.cos(
            time * math.sqrt(gravity * air_resistance_constant) - math.atan(
                velocity_up * math.sqrt(air_resistance_constant / gravity)))) - (
                           1 / air_resistance_constant) * math.log(
            math.cos(-math.atan(velocity_up * math.sqrt(air_resistance_constant / gravity))))
        return distance

    @staticmethod
    def time_down(distance):
        time = math.acosh(math.e ** (air_resistance_constant * distance)) / (
            math.sqrt(air_resistance_constant * gravity))
        return time

    @staticmethod
    def velocity_down(time):
        velocity = -math.sqrt(gravity / air_resistance_constant) * math.tanh(
            time * math.sqrt(gravity * air_resistance_constant))
        return velocity

    @staticmethod
    def velocity_horizontal(initial_velocity, time):
        velocity = initial_velocity / (initial_velocity * air_resistance_constant * time + 1)
        return velocity

    @staticmethod
    def initial_velocity_horizontal(distance, time):
        velocity = ((math.e ** (air_resistance_constant * distance)) - 1) / (air_resistance_constant * time)
        return velocity

    @staticmethod
    def distance_horizontal(velocity_horizontal, time):
        distance = (1 / air_resistance_constant) * math.log(velocity_horizontal * air_resistance_constant * time + 1)
        return distance

    @classmethod
    def calculate_required_velocity(cls, velocity_vertical, distance_to_hub):
        time1 = cls.time_up(velocity_vertical)
        dist_up = cls.distance_up(time1, velocity_vertical)
        dist_down = dist_up - height_difference
        time2 = cls.time_down(dist_down)
        total_time = time1 + time2

        required_initial_velocity = (cls.initial_velocity_horizontal(distance_to_hub, total_time), velocity_vertical)

        final_velocity = (cls.velocity_horizontal(required_initial_velocity[0], total_time), cls.velocity_down(time2))

        return required_initial_velocity, final_velocity

    @staticmethod
    def calculate_energy(velocity):
        energy = (velocity[0] ** 2) + (velocity[1] ** 2)
        return energy

    @staticmethod
    def calculate_angle(velocity):
        input_angle = math.atan2(velocity[1], velocity[0])
        return abs(ideal_entry_angle - input_angle)

    @classmethod
    def velocity_angle_minimize(cls, velocities, distance):
        final_vel = velocities[1]
        a, b = 1, 40 * ((distance / 9) ** 2)
        return a * cls.calculate_energy(final_vel) + b * cls.calculate_angle(final_vel)

    @staticmethod
    def energy_minimize(velocities, distance):
        initial_velocity = velocities[0]
        energy = (initial_velocity[0] ** 2) + (initial_velocity[1] ** 2)
        return energy

    @classmethod
    def stationary_aim(cls, distance_to_hub, velocity_vertical=7.3, direction=None, current_function_value=None,
                       step_size=0.1, function=None):
        """
        calculates how to orient the robot and what velocity to give the ball to make a shot while stationary


        distance_to_hub is the distance from the shooter to the hub in meters

        velocity_up is an initial guess for the vertical component of velocity

        step_size is the size of the steps in gradient descent


        The returned value is a tuple with the horizontal and vertical components of velocity
        """

        if function is None:
            function = cls.energy_minimize

        # Figure out the direction
        if direction is None:
            # Check the current function value. If that gives an error, we need to try a higher velocity
            try:
                current_function_value = function(cls.calculate_required_velocity(velocity_vertical, distance_to_hub), distance_to_hub)
            except ValueError:
                return cls.stationary_aim(distance_to_hub, velocity_vertical + step_size, direction,
                                          current_function_value, step_size, function)

            # Check the function value when the vertical component of velocity is increased
            increase_function_value = function(
                cls.calculate_required_velocity(velocity_vertical + step_size, distance_to_hub), distance_to_hub)

            # See if we should increase the vertical component of velocity
            if increase_function_value < current_function_value:
                return cls.stationary_aim(distance_to_hub, velocity_vertical + step_size, 1, increase_function_value,
                                          step_size,
                                          function)

            # Check the function value when we decrease the vertical component of velocity. If we can't, then we are
            # at a min.
            try:
                decrease_function_value = function(
                    cls.calculate_required_velocity(velocity_vertical - step_size, distance_to_hub), distance_to_hub)
            except ValueError:
                return cls.calculate_required_velocity(velocity_vertical, distance_to_hub)[0]

            # If it is better to decrease, then decrease
            if decrease_function_value < current_function_value:
                return cls.stationary_aim(distance_to_hub, velocity_vertical - step_size, -1, decrease_function_value,
                                          step_size, function)

            return cls.calculate_required_velocity(velocity_vertical, distance_to_hub)[0]

        # Go in the direction that was just found until it stops decreasing the function value or there is a math
        # domain error
        try:
            new_function_value = function(
                cls.calculate_required_velocity(velocity_vertical + (step_size * direction), distance_to_hub), distance_to_hub)
        except ValueError:
            return cls.calculate_required_velocity(velocity_vertical, distance_to_hub)[0]

        if new_function_value < current_function_value:
            return cls.stationary_aim(distance_to_hub, velocity_vertical + step_size * direction, direction,
                                      new_function_value, step_size, function)

        return cls.calculate_required_velocity(velocity_vertical, distance_to_hub)[0]

    @classmethod
    def moving_aim(cls, distance_to_hub, robot_velocity, velocity_up=10, step_size=0.1,
                   function=None):
        """
        calculates how to orient the robot and what velocity to give the ball to make a shot while moving


        distance_to_hub is the distance from the shooter to the hub in meters

        robot_velocity is a tuple with the velocity of the robot in m/s

        velocity_up is an initial guess for the vertical component of velocity

        step_size is the size of the steps in gradient descent


        The returned value is a tuple with the horizontal and vertical components of velocity, and an orientation of
        the robot in radians
        """

        if function is None:
            function = cls.velocity_angle_minimize

        # What velocity the ball needs to have
        required_velocity = cls.stationary_aim(distance_to_hub, velocity_up, step_size=step_size, function=function)

        shooter_setting = (
            numpy.linalg.norm((required_velocity[0] - robot_velocity[1], robot_velocity[0])), required_velocity[1])

        # The shooter cannot shoot below a certain angle. If the shooter cannot shoot the optimal angle, this loop
        # finds an angle it can shoot

        while math.atan2(shooter_setting[1], shooter_setting[0]) < minimum_shooter_angle:
            required_velocity = cls.calculate_required_velocity(required_velocity[1] + 1, distance_to_hub)[0]
            shooter_setting = (
                numpy.linalg.norm((required_velocity[0] - robot_velocity[1], robot_velocity[0])), required_velocity[1])

        # Catches 3 situations in which the shooter cannot shoot.
        # The angle is too steep, the velocity is too high, or the robot is moving too fast towards the hub
        if math.atan2(shooter_setting[1], shooter_setting[0]) < minimum_shooter_angle or numpy.linalg.norm(
                shooter_setting) > max_shooter_velocity or required_velocity[0] < robot_velocity[1]:
            return None, None

        orientation_setting = math.atan(robot_velocity[0] / (required_velocity[0] - robot_velocity[1]))

        return shooter_setting, orientation_setting

    @staticmethod
    def convert_position(robot_angle, new_position):
        """
        converts between the different coordinate systems used for shooting


        robot_angle is the current orientation of the robot in radians

        new_position is the new position in the original coordinate system as a tuple

        returns a tuple containing the new distance to the hub (in meters) and the new angle (in radians)

        """

        new_dist = numpy.linalg.norm(new_position)

        angle_to_hub = math.atan2(new_position[0], -new_position[1])

        new_angle = robot_angle - angle_to_hub

        return new_dist, new_angle

    @staticmethod
    def convert_velocity(new_position, velocity):
        """
        converts velocity from an old coordinate system into a new coordinate system


        new_position is the new position of the robot in the current coordinate system

        velocity is the current velocity of the robot in the current coordinate system


        returns the new velocity in the new coordinate system
        """

        position_magnitude = numpy.linalg.norm(new_position)

        complex_rotation = -complex(new_position[1] / position_magnitude, new_position[0] / position_magnitude)

        complex_velocity = complex(velocity[0], velocity[1])
        new_complex_velocity = complex_velocity * complex_rotation
        new_velocity = (new_complex_velocity.real, new_complex_velocity.imag)

        return new_velocity

    @staticmethod
    def new_position(distance_to_hub, velocity, time=1):
        """
        calculates the expected new position of the robot given a constant velocity


        distance_to_hub is the distance from the hub,

        velocity is a tuple of the velocity in the shooter coordinate system

        time is the amount of time in the future that the robot will be in this new position


        returns a tuple with position vector in the shooter coordinate system
        """

        new_position = (velocity[0] * time, (velocity[1] * time) - distance_to_hub)

        return new_position

    @classmethod
    def goal_angle_to_current(cls, goal_angle, new_position, current_angle):
        """
        calculates how much the robot needs to rotate to achieve the goal angle


        goal_angle is the goal angle in radians

        current_angle is the current angle in radians

        new_position is the new position as tuples in the current coordinate system


        returns an angle to rotate in radians
        """
        if goal_angle is None:
            return None

        # If we don't rotate, what will the angle be?
        future_angle = cls.convert_position(current_angle, new_position)[1]

        # How far off is this from what we want?
        angle_difference = goal_angle - future_angle

        return angle_difference

    @classmethod
    def moving_aim_ahead(cls, current_angle, robot_velocity, distance_to_hub, time=shooter_delay, guess_velocity=10, step_size=0.1):
        """
        calculates how to orient the robot and prepare the shooter to shoot in a given amount of time


        current_angle is the current angle of the robot in radians

        robot_velocity is the current velocity of the robot in m/s

        distance_to_hub is the current distance to the hub in meters


        returns a tuple with the velocity the shooter should give to the ball, and an angle for the robot to rotate
        """
        new_pos = cls.new_position(distance_to_hub, robot_velocity, time)
        future_situation = cls.convert_position(current_angle, new_pos)
        future_velocity = cls.convert_velocity(new_pos, robot_velocity)

        aim = cls.moving_aim(future_situation[0], future_velocity, guess_velocity, step_size)
        shooter_setting = aim[0]
        need_angle = aim[1]
        rotate = cls.goal_angle_to_current(need_angle, new_pos, current_angle)

        return shooter_setting, rotate

    @classmethod
    def should_shoot(cls, current_angle, robot_velocity, distance_to_hub, shooter_setting, time=0):
        """
        boolean function that determines if now is a good time to begin the shooting process


        current_angle is the current angle of the robot in radians
        robot_velocity is the current velocity of the robot in m/s
        distance_to_hub is the current distance to the hub in meters
        shooter_setting is the current setting of the shooter
        time is how far in the future this function is looking

        returns a boolean that is true if now is a good time to shoot, and false if it is not a good time to shoot
        """
        time += shooter_delay

        new_pos = cls.new_position(distance_to_hub, robot_velocity, time)
        future_angle = cls.convert_position(current_angle, new_pos)[1]
        future_velocity = cls.convert_velocity(new_pos, robot_velocity)

        time1 = cls.time_up(shooter_setting[1])
        dist_up = cls.distance_up(time1, shooter_setting[1])
        dist_down = dist_up - height_difference
        time2 = cls.time_down(dist_down)
        total_time = time1 + time2

        top_down_ball_velocity = numpy.add(future_velocity, (
            -shooter_setting[0] * math.sin(future_angle), shooter_setting[0] * math.cos(future_angle)))

        final_ball_position = (
            (1 / air_resistance_constant) * math.log(
                top_down_ball_velocity[0] * air_resistance_constant * total_time + 1),
            (1 / air_resistance_constant) * math.log(
                top_down_ball_velocity[1] * air_resistance_constant * total_time + 1))
        goal_ball_position = (0, cls.convert_position(current_angle, new_pos)[0])

        ball_error = numpy.linalg.norm(numpy.subtract(final_ball_position, goal_ball_position))

        return ball_error <= acceptable_error

    @staticmethod
    def real_velocity_to_shooting(real_velocity, angle_to_hub):
        """
        converts the actual velocity of the robot into the velocity in the coordinate system used for shooting


        real_velocity is the velocity of the robot where the front of the robot is in the +y direction (m/s)

        angle_to_hub is the angle between the shooter and the hub (rad)


        returns a new velocity tuple in the coordinate system used for shooting
        """

        robot_to_hub = complex(math.cos(angle_to_hub), -math.sin(angle_to_hub))

        complex_velocity = complex(real_velocity[0], real_velocity[1])

        complex_rotated_velocity = complex_velocity / robot_to_hub

        rotated_velocity = (complex_rotated_velocity.real, complex_rotated_velocity.imag)

        return rotated_velocity

    @classmethod
    def launch_up(cls, max_height, distance):
        # This uses calculus to find the optimal shooting angle, however it is typically way too steep
        # best_upwards_velocity = math.sqrt((gravity / air_resistance_constant)) * math.tan(
        #     math.acos(math.e ** (-air_resistance_constant * max_height)))
        # best_velocity = calculate_required_velocity(best_upwards_velocity, distance, False)[0]
        # while math.atan2(best_velocity[1], best_velocity[0]) > max_shooter_angle:
        #     best_velocity = calculate_required_velocity(best_velocity[1] - 0.1, distance, False)[0]
        # return best_velocity
        step = 0.1

        speed = math.sqrt(distance * gravity / (math.cos(max_shooter_angle) * math.sin(max_shooter_angle) * 2))
        no_air_velocity = numpy.multiply((math.cos(max_shooter_angle), math.sin(max_shooter_angle)), speed)
        best_velocity = cls.calculate_required_velocity(no_air_velocity[1], distance, False)[0]

        if cls.distance_up(cls.time_up(best_velocity[1]), best_velocity[1]) > max_height:
            while cls.distance_up(cls.time_up(best_velocity[1]), best_velocity[1]) > max_height:
                best_velocity = cls.calculate_required_velocity(best_velocity[1] - step, distance, False)[0]
            return best_velocity

        potential_velocity = best_velocity

        while math.atan2(potential_velocity[1], potential_velocity[0]) < max_shooter_angle and cls.distance_up(
                cls.time_up(best_velocity[1]), best_velocity[1]) < max_height:
            best_velocity = potential_velocity
            potential_velocity = cls.calculate_required_velocity(best_velocity[1] + step, distance, False)[0]
        return best_velocity
