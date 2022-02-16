from networktables import NetworkTables
import math


class Limelight:

    # all possible info from limelight network tables
    # tx : horizontal angle away from center (-27 to 27 degrees)
    # ty : vertical angle away from the center (-20.5 to 20.5 degrees)
    # ta : Area in percentage of image of the reflective surface that the limelight detects (pixels?)
    # ts : skew or rotation of the reflective surface from (-90 to 0 degrees)

    HUB_HEIGHT = 104  # units inches (where is the upper hub from ground: 8 feet 8 inches)


    def __init__(self, cam_height, cam_angle, dist_from_center):
        NetworkTables.initialize()
        self.table = NetworkTables.getTable("limelight")
        self.cam_height = cam_height  # units inches (height from ground to camera)
        self.cam_angle = cam_angle  # units radians (angle from horizontal)
        self.dist_from_center = dist_from_center

    def get_distance(self):
        # values for calculation
        angle_y = self.table.getNumber('ty', None)

        if angle_y is None:
            return -1

        h_hub_angle = math.radians(angle_y)  # convert to radians

        distance = (self.HUB_HEIGHT - self.cam_height) / math.tan(self.cam_angle + h_hub_angle)
        return distance + self.dist_from_center

    def get_orientation_offset(self):
        return self.table.getNumber('tx', None)

