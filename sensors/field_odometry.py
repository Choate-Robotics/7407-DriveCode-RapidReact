import math

from networktables import NetworkTables
from robotpy_toolkit_7407.utils.units import meters, radians


class FieldOdometry:
    def __init__(self):
        self.hub_angle: radians | None = None
        self.hub_dist: meters | None = None

        NetworkTables.initialize()
        self._limelight = NetworkTables.getTable("limelight")
        self._l_tx: radians | None = None
        self._l_ty: radians | None = None
        self._l_dist: meters | None = None

    def update(self):
        self._collect_limelight_data()
        self.hub_angle = self._l_tx
        self.hub_dist = self._l_dist

    def _collect_limelight_data(self):
        self._l_tx = math.radians(self._limelight.getNumber('tx', None))
        self._l_ty = math.radians(self._limelight.getNumber('ty', None))
        self._l_dist = self._calculate_limelight_distance()

    def _calculate_limelight_distance(self) -> meters | None:
        if self._l_ty is None:
            return None
        true_angle = math.radians(43) + self._l_ty  # Camera angle
        distance = (2.6416 - 0.813) / math.tan(true_angle)  # Hub height minus camera height
        return distance
