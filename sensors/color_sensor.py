import wpilib
import rev

class Color_Sensor:
    def __init__(self, I2C_Port, deviceAddress): # TODO: Find device address
        self.color_sensor = rev.ColorSensorV3(wpilib.I2C(I2C_Port, deviceAddress))
    def get_color(self):
        return self.color_sensor.getColor()