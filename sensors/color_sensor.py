import wpilib
import rev

class Color_Sensor:
    def __init__(self, I2C_Port, lane): # TODO: Find device address
        self.lane = lane
        self.multiplexer = wpilib.I2C(I2C_Port, 0x70)  # TODO find this fr
        self.color_sensor = rev.ColorSensorV3(wpilib.I2C(I2C_Port, 0x52))
    def get_color(self):
        self.multiplexer.write(1 << self.lane)
        return self.color_sensor.getColor()