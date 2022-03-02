from math import degrees, pi
from networktables import NetworkTables
import math
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import m, deg, ft, inch, rad


class Limelight:
    def __init__(self):
        NetworkTables.initialize()
        self.table = NetworkTables.getTable("limelight")
        self.measure_amount = 3
        self.data_x = [0]
        self.data_y = [0]
        self.tx = 0
        self.ty = 0
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

    def update(self):
        c_tx = self.table.getNumber('tx', None)
        c_ty = self.table.getNumber('ty', None)
        if c_tx is None or c_ty is None:
            return
        self.data_x.append(c_tx)
        self.data_y.append(c_ty)
        if len(self.data_x) > self.measure_amount:
            self.data_x.pop(0)
            self.data_y.pop(0)
        # self.tx = sum(self.data_x) / len(self.data_x)
        # self.ty = sum(self.data_y) / len(self.data_y)
        self.tx = c_tx
        self.ty = c_ty

    def calculate_distance(self) -> float:
        # values for calculation
        cam_height = .813 * m  # units meters (height from ground to camera)
        cam_angle = 43 * deg  # units radians (angle from horizontal)
        h_hub_height = 8 * ft + 8 * inch  # units meters (where is the upper hub from ground) 8 feet 8 inches

        true_angle = cam_angle + self.ty * deg

        logger.info(f"true theta: {true_angle.asUnit(deg)}")
        
        distance = (h_hub_height - cam_height) / math.tan(true_angle.asNumber(rad))

        return distance.asNumber(m)

    def get_x_offset(self) -> float:
        return self.tx
