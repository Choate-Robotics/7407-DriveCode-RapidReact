import math

from networktables import NetworkTables
from robotpy_toolkit_7407.utils.units import m, deg, ft, inch, rad, radians, meters


class Limelight:
    def __init__(self):
        NetworkTables.initialize()
        self.table = NetworkTables.getTable("limelight")
        self.tx = 0
        self.ty = 0
        self.refs = 0
        self.k_cam_height: meters = .864  # Height from ground to camera # .813
        self.k_cam_angle: radians = (43 * deg).asNumber(rad)  # Angle from horizontal # 43 45 48 38
        self.k_h_hub_height = (8 * ft + 8 * inch).asNumber(m)

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
        self.tx = c_tx
        self.ty = c_ty

    def calculate_distance(self) -> float:
        true_angle = self.k_cam_angle + self.ty
        distance = (self.k_h_hub_height - self.k_cam_height) / math.tan(true_angle)
        return distance

    def get_x_offset(self) -> radians:
        return math.radians(self.tx)
