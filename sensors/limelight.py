from math import degrees, pi
from networktables import NetworkTables
import math
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import m, deg, ft, inch, rad


class Limelight():
    
    '''
    tx = table.getNumber('tx',None)  # horizontal angle away from center (-27 to 27 degrees)
    ty = table.getNumber('ty',None)  # vertical angle away from the center (-20.5 to 20.5 degrees)
    ta = table.getNumber('ta',None)  # Area in percentage of image of the reflective surface that the limelight detects (pixels?)
    ts = table.getNumber('ts',None)  # skew or rotation of the reflective surface from (-90 to 0 degrees)
    '''
    
    def __init__(self):
        NetworkTables.initialize()
        self.table = NetworkTables.getTable("limelight")
        
    def calculate_distance(self) -> float:
        # values for calculation
        cam_height = .813 * m  # units meters (height from ground to camera)
        cam_angle = 43 * deg  # units radians (angle from horizontal)
        h_hub_height = 8 * ft + 8 * inch  # units meters (where is the upper hub from ground) 8 feet 8 inches
        ty = self.table.getNumber('ty', 0) * deg
        if ty is None:
            return -1

        true_angle = cam_angle + ty

        logger.info(f"true theta: {true_angle.asUnit(deg)}")
        
        distance = (h_hub_height - cam_height) / math.tan(true_angle.asNumber(rad))

        return distance.asNumber(m)

    def get_x_offset(self) -> float:
        return math.radians(self.table.getNumber('tx', None))
