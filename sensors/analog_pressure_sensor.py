import wpilib
import ctre
import rev

class Analog_Pressure_Sensor:
    def __init__(self, CanAddress, PressureChannel): # TODO: Find device address
        self.pneumatic_hub = wpilib.PneumaticHub(CanAddress)
        self.pressure_channel = PressureChannel
    def get_pressure(self):
        return self.pneumatic_hub.getPressure(self.pressure_channel)