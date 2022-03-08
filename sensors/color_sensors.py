from rev import ColorSensorV3
from wpilib import I2C


class ColorSensor:
    def __init__(self):
        self.multiplexer = I2C(I2C.Port.kMXP, 0x71)
        self.multiplexer.writeBulk(bytes([0b100]))
        self.sensor = ColorSensorV3(I2C.Port.kMXP)

    def get_val(self) -> tuple[float, float, float, int]:
        c = self.sensor.getRawColor()
        return c.red, c.green, c.blue, self.sensor.getProximity()

    def color(self) -> str:
        vals = self.get_val()
        if vals[0]-vals[2]>100:
            return "red"
        elif vals[2]-vals[0]>100:
            return "blue"
        return "none"
