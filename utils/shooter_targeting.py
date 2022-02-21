import math
from constants import air_resistance_constant, height_difference, gravity

class Shooter_Targeting:

    def __init__(self):
        pass

    def time_up(self, velocity_up):
        time = math.atan(velocity_up*math.sqrt(air_resistance_constant/gravity))/(math.sqrt(gravity*air_resistance_constant))
        return time


    def distance_up(self, time, velocity_up):
        distance = (1/air_resistance_constant)*math.log(math.cos(time*math.sqrt(gravity*air_resistance_constant)-math.atan(velocity_up*math.sqrt(air_resistance_constant/gravity))))-(1/air_resistance_constant)*math.log(math.cos(-math.atan(velocity_up*math.sqrt(air_resistance_constant/gravity))))

        return distance


    def time_down(self, distance):
        time = math.acosh(math.e**(air_resistance_constant*distance))/(math.sqrt(air_resistance_constant*gravity))
        return time


    def velocity_horizontal(self, distance, time):
        velocity = ((math.e**(air_resistance_constant*distance))-1)/(air_resistance_constant*time)
        return velocity


    def calculate_velocity(self, velocity_up, distance_to_hub):
        time1 = self.time_up(velocity_up)
        dist_up = self.distance_up(time1, velocity_up)
        dist_down = dist_up - height_difference
        time2 = self.time_down(dist_down)
        total_time = time1 + time2
        velocity_sideways = self.velocity_horizontal(distance_to_hub, total_time)

        return velocity_sideways, velocity_up


    def calculate_energy(self, velocity):
        energy = (velocity[0]**2)+(velocity[1]**2)

        return energy


    def gradient_velocity(self, distance_to_hub, velocity_up=10.0, step_size=0.1):
        try:
            current_energy = self.calculate_energy(self.calculate_velocity(velocity_up, distance_to_hub))
            increase_energy = self.calculate_energy(self.calculate_velocity(velocity_up + step_size, distance_to_hub))
            decrease_energy = self.calculate_energy(self.calculate_velocity(velocity_up - step_size, distance_to_hub))

            if increase_energy < current_energy:
                return self.gradient_velocity(distance_to_hub, velocity_up + step_size, step_size)

            if decrease_energy < current_energy:
                return self.gradient_velocity(distance_to_hub, velocity_up - step_size, step_size)

            return self.calculate_velocity(velocity_up, distance_to_hub)
        except:
            return self.gradient_velocity(distance_to_hub, velocity_up + step_size, step_size)
