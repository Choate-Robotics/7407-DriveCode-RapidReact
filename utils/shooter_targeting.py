import math, numpy
from constants import air_resistance_constant, height_difference, gravity, acceptable_error, shooter_delay


class ShooterTargeting:
    @staticmethod
    def time_up(velocity_up):
        time = math.atan(velocity_up*math.sqrt(air_resistance_constant/gravity))/(math.sqrt(gravity*air_resistance_constant))
        return time

    @staticmethod
    def distance_up(time, velocity_up):
        distance = (1/air_resistance_constant)*math.log(math.cos(time*math.sqrt(gravity*air_resistance_constant)-math.atan(velocity_up*math.sqrt(air_resistance_constant/gravity))))-(1/air_resistance_constant)*math.log(math.cos(-math.atan(velocity_up*math.sqrt(air_resistance_constant/gravity))))
        return distance

    @staticmethod
    def time_down(distance):
        time = math.acosh(math.e**(air_resistance_constant*distance))/(math.sqrt(air_resistance_constant*gravity))
        return time

    @staticmethod
    def velocity_horizontal(distance, time):
        velocity = ((math.e**(air_resistance_constant*distance))-1)/(air_resistance_constant*time)
        return velocity

    @classmethod
    def calculate_velocity(cls, velocity_up, distance_to_hub):
        time1 = cls.time_up(velocity_up)
        dist_up = cls.distance_up(time1, velocity_up)
        dist_down = dist_up - height_difference
        time2 = cls.time_down(dist_down)
        total_time = time1 + time2
        velocity_sideways = cls.velocity_horizontal(distance_to_hub, total_time)

        return velocity_sideways, velocity_up

    @staticmethod
    def calculate_energy(velocity):
        energy = (velocity[0]**2)+(velocity[1]**2)
        return energy

    @classmethod
    def stationary_aim(cls, distance_to_hub, velocity_up=10, step_size=0.1):
        """
        calculates how to orient the robot and what velocity to give the ball to make a shot while stationary


        distance_to_hub is the distance from the shooter to the hub in meters

        velocity_up is an initial guess for the vertical component of velocity

        step_size is the size of the steps in gradient descent


        The returned value is a tuple with the horizontal and vertical components of velocity
        """

        try:
            current_energy = cls.calculate_energy(cls.calculate_velocity(velocity_up, distance_to_hub))
        except:
            # If the ball doesn't go high enough with this velocity, try a higher one
            return cls.stationary_aim(distance_to_hub, velocity_up + step_size, step_size)

        increase_energy = cls.calculate_energy(cls.calculate_velocity(velocity_up + step_size, distance_to_hub))

        # If increasing the velocity up yields lower energy, then increase velocity up
        if increase_energy < current_energy:
            return cls.stationary_aim(distance_to_hub, velocity_up + step_size, step_size)

        # If increasing the velocity up does not yield lower energy, then either we are at a minimum or we need to decrease
        try:
            decrease_energy = cls.calculate_energy(cls.calculate_velocity(velocity_up - step_size, distance_to_hub))
        except:
            # If we cannot decrease the velocity up, then we have reached a minimum
            return cls.calculate_velocity(velocity_up, distance_to_hub)

        # If decreasing the velocity up will yield lower energy, then we should decrease the velocity up
        if decrease_energy < current_energy:
            return cls.stationary_aim(distance_to_hub, velocity_up - step_size, step_size)

        # If it is not better to increase and it is not better to decrease, then we have reached a min
        return cls.calculate_velocity(velocity_up, distance_to_hub)

    @classmethod
    def moving_aim(cls,distance_to_hub, robot_velocity, velocity_up=10, step_size=0.1):
        """
        calculates how to orient the robot and what velocity to give the ball to make a shot while moving


        distance_to_hub is the distance from the shooter to the hub in meters

        robot_velocity is a tuple with the velocity of the robot in m/s

        velocity_up is an initial guess for the vertical component of velocity

        step_size is the size of the steps in gradient descent


        The returned value is a tuple with the horizontal and vertical components of velocity, and an orientation of the robot in radians
        """
        required_velocity = cls.stationary_aim(distance_to_hub, velocity_up, step_size)
        shooter_setting = (
        math.sqrt((required_velocity[0] - robot_velocity[1]) ** 2 + robot_velocity[0] ** 2), required_velocity[1])
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

        return (new_dist, new_angle)

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
    def goal_angle_to_current(cls,goal_angle, new_position, current_angle):
        """
        calculates how much the robot needs to rotate to acheive the goal angle


        goal_angle is the goal angle in radians

        current_angle is the current angle in radians

        new_position is the new position as tuples in the current coordinate system


        returns an angle to rotate in radians
        """

        # If we don't rotate, what will the angle be?
        future_angle = cls.convert_position(current_angle, new_position)[1]

        # How far off is this from what we want?
        angle_difference = goal_angle - future_angle

        return angle_difference

    @classmethod
    def moving_aim_ahead(cls,current_angle, robot_velocity, distance_to_hub, time=1, guess_velocity=10, step_size=0.1):
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
    def should_shoot(cls,current_angle, robot_velocity, distance_to_hub, shooter_setting, time=shooter_delay):
        """
        boolean function that determines if now is a good time to begin the shooting process


        current_angle is the current angle of the robot in radians
        robot_velocity is the current velocity of the robot in m/s
        distance_to_hub is the current distance to the hub in meters
        shooter_setting is the current setting of the shooter
        time is how far in the future this function is looking

        returns a boolean that is true if now is a good time to shoot, and false if it is not a good time to shoot
        """
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
        (1 / air_resistance_constant) * math.log(top_down_ball_velocity[0] * air_resistance_constant * total_time + 1),
        (1 / air_resistance_constant) * math.log(top_down_ball_velocity[1] * air_resistance_constant * total_time + 1))
        goal_ball_position = (0, cls.convert_position(current_angle, new_pos)[0])

        ball_error = numpy.linalg.norm(numpy.subtract(final_ball_position, goal_ball_position))

        return ball_error <= acceptable_error

    @staticmethod
    def real_velocity_to_shooting(real_velocity, robot_orientation, angle_to_hub):
        """
        converts the actual velocity of the robot into the velocity in the coordinate system used for shooting


        real_velocity is the velocity of the robot as it is used in driving (m/s)

        robot_orientation is how much the robot has rotated from an objective starting direction (rad)

        angle_to_hub is the angle between the shooter and the hub (rad)


        returns a new velocity tuple in the coordinate system used for shooting
        """

        objective_to_robot = complex(math.cos(robot_orientation), math.sin(robot_orientation))

        robot_to_hub = complex(math.cos(angle_to_hub), -math.sin(angle_to_hub))

        # total_rotation is how much the new coordinate system has been rotated compared to the objective coordinate system
        total_rotation = objective_to_robot * robot_to_hub

        complex_velocity = complex(real_velocity[0], real_velocity[1])

        complex_rotated_velocity = complex_velocity / total_rotation

        rotated_velocity = (complex_rotated_velocity.real, complex_rotated_velocity.imag)

        return rotated_velocity