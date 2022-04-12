from math import degrees, pi
from networktables import NetworkTables
import math
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import m, deg, ft, inch, rad


class Limelight:
    def __init__(self):
        NetworkTables.initialize()
        self.table = NetworkTables.getTable("limelight")
        self.refs = 0

    def led_on(self):
        self.table.putNumber("ledMode", 3)

    def led_off(self):
        self.table.putNumber("ledMode", 1)

    def ref_on(self):
        if self.refs == 0:
            self.led_on()
        self.refs += 1

    def ref_off(self):
        self.refs -= 1
        if self.refs == 0:
            self.led_off()

    def get_angles(self):
        return self.table.getNumber('tx', None), self.table.getNumber('ty', None)
