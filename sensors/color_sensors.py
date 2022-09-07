from rev import ColorSensorV3
from wpilib import I2C
#import time


class ColorSensors:
    def __init__(self):
        self.multiplexer = I2C(I2C.Port.kMXP, 0x71)
        self.multiplexer.writeBulk(bytes([0b0100]))
        self.sensor = ColorSensorV3(I2C.Port.kMXP)
        self.multiplexer.writeBulk(bytes([0b1000]))
        self.sensor = ColorSensorV3(I2C.Port.kMXP)
        self.working = "ples_enable"

    def get_val(self) -> tuple[float, float, float, int]:
        c = self.sensor.getRawColor()
        return c.red, c.green, c.blue, self.sensor.getProximity()

    def color(self) -> str:
        vals = self.get_val()
        if vals[0] == 0:
            self.multiplexer = I2C(I2C.Port.kMXP, 0x71)
            self.sensor = ColorSensorV3(I2C.Port.kMXP)
        if vals[0]-vals[2]>500:
            return "red"
        elif vals[2]-vals[0]>500 and vals[1]>400:
            return "blue"
        return "none"

    def get_color_left(self) -> str:
        self.multiplexer.writeBulk(bytes([0b1000]))
        #time.sleep(.02)
        vals = self.get_val()
        if vals[0]-vals[2]>2000:
            return "red"
        elif vals[2]-vals[0]>2000:
            return "blue"
        return "none"
    def get_color_right(self) -> str:
        self.multiplexer.writeBulk(bytes([0b100]))
        vals = self.get_val()
        if vals[0]-vals[2]>2000:
            return "red"
        elif vals[2]-vals[0]>2000:
            return "blue"
        return "none"

    def get_val_left(self) -> tuple:
        self.multiplexer.writeBulk(bytes([0b1000]))
        return self.get_val()

    def get_val_right(self) -> tuple:
        self.multiplexer.writeBulk(bytes([0b100]))
        return self.get_val()
