import math
from constants import air_resistance_constant, height_difference, gravity


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
