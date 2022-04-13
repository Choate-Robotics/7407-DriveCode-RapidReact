import math
import time

from networktables import NetworkTables
from robotpy_toolkit_7407.utils.units import meters, radians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from subsystem import Drivetrain


class FieldOdometry:
    def __init__(self, drivetrain: Drivetrain):
        self.drivetrain = drivetrain

        self.robot_pose: Pose2d | None = None

        self.hub_angle: radians | None = None
        self.hub_dist: meters | None = None

        NetworkTables.initialize()
        self._limelight = NetworkTables.getTable("limelight")
        self._l_tx: radians | None = None
        self._l_ty: radians | None = None
        self._l_dist: meters | None = None

        self._hub_pose = Pose2d(8.56, 4.196, 0)

        self.last_update_time = None
        self.min_update_wait_time = 5

        self._led_on()

    def update(self):
        self._collect_limelight_data()
        self.robot_pose = Pose2d(
            self.drivetrain.odometry.getPose().translation(),
            Rotation2d(self.drivetrain.gyro.get_robot_heading())
        )
        self._calc_values_from_pose()

        t = time.time()
        if self.last_update_time is None or t > self.last_update_time + self.min_update_wait_time:
            new_pose = self._calc_pose_from_limelight(self.robot_pose.rotation())
            if new_pose is not None:
                self.drivetrain.odometry.resetPosition(new_pose, self.robot_pose.rotation())
            self.last_update_time = t
            print(f"hub_dist={self.hub_dist}, limelight_angle={self.hub_angle}")
            print(f"new_pose={new_pose}, limelight_vals={self._l_tx, self._l_dist}")

    def _calc_values_from_pose(self):
        offset = self._hub_pose.relativeTo(self.robot_pose)

        d_out = offset.Y()
        d_horizontal = offset.X()

        self.hub_dist = math.sqrt(d_out**2 + d_horizontal**2)
        self.hub_angle = math.atan2(d_horizontal, d_out)

    def _calc_pose_from_limelight(self, gyro_angle: Rotation2d) -> Pose2d | None:
        if self._l_dist is None or self._l_tx is None:
            return None

        r_translation = Translation2d(
            self._l_dist * math.sin(self._l_tx),
            self._l_dist * math.cos(self._l_tx)
        ).rotateBy(gyro_angle)  # Might have to flip direction of rotation

        return Pose2d(
            self._hub_pose.X() - r_translation.X(),
            self._hub_pose.Y() - r_translation.Y(),
            gyro_angle
        )

    def _collect_limelight_data(self):
        self._l_tx = self._limelight.getNumber('tx', None)
        if self._l_tx:
            self._l_tx = math.radians(self._l_tx)
        else:
            self._l_tx = None

        self._l_ty = self._limelight.getNumber('ty', None)
        if self._l_ty:
            self._l_ty = math.radians(self._l_ty)
        else:
            self._l_ty = None

        self._l_dist = self._calculate_limelight_distance()

    def _calculate_limelight_distance(self) -> meters | None:
        if self._l_ty is None:
            return None
        true_angle = math.radians(43) + self._l_ty  # Camera angle
        distance = (2.6416 - 0.813) / math.tan(true_angle)  # Hub height minus camera height
        distance += 2.55-1.68
        return distance

    def _led_on(self):
        self._limelight.putNumber("ledMode", 3)

    def _led_off(self):
        self._limelight.putNumber("ledMode", 1)
