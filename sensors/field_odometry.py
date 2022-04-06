import math

from robotpy_toolkit_7407.unum import Unum
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import m, deg, inch, ft, rad
from wpimath.geometry import Pose2d, Rotation2d

from sensors import Limelight
from subsystem import Drivetrain


class AveragedDataField:
    def __init__(self, samples: int = 4):
        self.data = []
        self.samples = samples
        self.val = None

    def push(self, v: float):
        self.data.append(v)
        if len(self.data) > self.samples:
            self.data.pop(0)
        try:
            self.val = sum(self.data) / len(self.data)
        except TypeError as e:
            self.val = None


class FieldOdometry:
    def __init__(self, limelight: Limelight, drivetrain: Drivetrain, limelight_samples: int = 4):
        self.limelight_tx = AveragedDataField(limelight_samples)
        self.limelight_ty = AveragedDataField(limelight_samples)
        self.hub_pos = (16.4592 / 2, (8.2296 / 2) - 8.2296)
        self.limelight = limelight
        self.drivetrain = drivetrain
        self.dist_offset = 0.78

    def update(self):
        c_tx, c_ty = self.limelight.get_angles()
        self.limelight_tx.push(c_tx)
        self.limelight_ty.push(c_ty)
        self.update_pose_with_limelight_dist()

    def get_angle_to_hub(self) -> Unum:
        # If we see the limelight return that angle
        if tx := self.limelight_tx.val:
            return tx * deg

        # Otherwise, resort to robot pose
        robot_pose = self.drivetrain.odometry.getPose()
        raw_angle_to_limelight = math.atan2(
            self.hub_pos[1] - robot_pose.Y(),
            self.hub_pos[0] - robot_pose.X()
        )
        robot_orientation = robot_pose.rotation().rotateBy(Rotation2d(math.pi / 2))
        print(f"orientation={robot_orientation}")
        print(f"limelight_angle={raw_angle_to_limelight}")
        return robot_orientation.rotateBy(Rotation2d(-raw_angle_to_limelight)).radians() * rad

    def get_dist_to_hub(self) -> float:
        if hub_dist := self.limelight_dist_to_hub():
            return hub_dist - self.dist_offset
        return self.pose_dist_to_hub() - self.dist_offset

    def update_pose_with_limelight_dist(self):
        if limelight_angle := self.limelight_tx.val:
            if hub_dist := self.limelight_dist_to_hub():
                # Calculate field-relative angle to hub
                robot_pose = self.drivetrain.odometry.getPose()
                robot_angle = robot_pose.rotation().rotateBy(Rotation2d(math.pi / 2)).radians()
                angle_to_hub = robot_angle - math.radians(limelight_angle)

                # X and Y displacement from robot to hub
                dx, dy = hub_dist * math.cos(angle_to_hub), hub_dist * math.sin(angle_to_hub)

                new_pose = Pose2d(self.hub_pos[0] - dx, self.hub_pos[1] - dy, Rotation2d(robot_angle).rotateBy(Rotation2d(-math.pi / 2)))

                # LOGGING:
                # logger.info(f"limelight_pose = {new_pose}")
                # logger.info(f"tx             = {limelight_angle}")
                # logger.info(f"odometry_pose  = {robot_pose}")
                # logger.info(f"diff = {(new_pose.X() - robot_pose.X(), new_pose.Y() - robot_pose.Y())}")
                # logger.info("----------------------------------")
                # odometry_hub_dist = math.sqrt(
                #     (robot_pose.X() - self.hub_pos[0])**2 + (robot_pose.Y() - self.hub_pos[1])**2
                # )
                # logger.info(f"odometry_hub_dist = {odometry_hub_dist}")
                # logger.info(f"limelight_hub_dist = {hub_dist}")
                # logger.info(f"diff = {odometry_hub_dist - hub_dist}")
                # logger.info("--------------------------------------------------------------------")

                # ACTUAL UPDATING:
                # self.drivetrain.odometry.resetPosition(
                #     new_pose,
                #     Rotation2d(self.drivetrain.gyro.get_robot_heading().asNumber(rad))
                # )
                pass

        print(self.get_angle_to_hub().asUnit(deg))

    def limelight_dist_to_hub(self) -> float | None:
        if angle := self.limelight_ty.val:
            cam_height = .813 * m  # height from ground to camera
            cam_angle = 43 * deg  # angle from horizontal
            h_hub_height = 8 * ft + 8 * inch  # where is the upper hub from ground

            true_angle = cam_angle + angle * deg

            distance = (h_hub_height - cam_height) / math.tan(true_angle.asNumber(rad))

            return distance.asNumber(m) + self.dist_offset
        return None

    def pose_dist_to_hub(self) -> float:
        pose = self.drivetrain.odometry.getPose()
        dx, dy = pose.X() - self.hub_pos[0], pose.Y() - self.hub_pos[1]
        return math.sqrt(dx*dx + dy*dy)
