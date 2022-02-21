from rev import ColorSensorV3
from wpilib import I2C


class ColorSensors:
    def __init__(self):
        self.multiplexer = I2C(I2C.Port.kMXP, 0x71)
        self.multiplexer.writeBulk(bytes([0b100]))
        self.sensor = ColorSensorV3(I2C.Port.kMXP)

    def get_val(self) -> tuple[float, float, float, int]:
        c = self.sensor.getRawColor()
        return c.red, c.green, c.blue, self.sensor.getProximity()
