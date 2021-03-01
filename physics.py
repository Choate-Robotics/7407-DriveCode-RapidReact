import math

from pyfrc.physics.core import PhysicsInterface
from wpimath.geometry import Pose2d, Transform2d, Translation2d, Rotation2d

import constants
import subsystem


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface):
        self.physics_controller = physics_controller
        self.physics_controller.field.setRobotPose(subsystem.SimDrivetrain.POSE)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.
        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        subsystem.SimDrivetrain.POSE = self.invert_pose(self.physics_controller.get_pose())

        old_pose = subsystem.SimDrivetrain.POSE

        new_pose = self.update_pose(old_pose, tm_diff,
                                    subsystem.SimDrivetrain.LEFT_VEL_METERS_PER_SECOND,
                                    subsystem.SimDrivetrain.RIGHT_VEL_METERS_PER_SECOND)

        self.physics_controller.field.setRobotPose(self.invert_pose(new_pose))

    @staticmethod
    def invert_pose(old_pose: Pose2d) -> Pose2d:
        return Pose2d(old_pose.X(), -old_pose.Y(), -old_pose.rotation().radians())

    @staticmethod
    def update_pose(old_pose: Pose2d, dt: float, left_vel: float, right_vel: float):
        if left_vel == right_vel:
            if left_vel == 0:
                return old_pose
            translation = Translation2d(left_vel * dt, 0).rotateBy(old_pose.rotation())
            new_position = old_pose.transformBy(Transform2d(translation, Rotation2d(0)))
        elif left_vel == -right_vel:
            dist = left_vel * dt
            theta = 2 * dist / constants.track_width_meters
            new_position = Pose2d(old_pose.translation(), old_pose.rotation().rotateBy(Rotation2d(theta)))
        else:
            vel = 0.5 * (left_vel + right_vel)
            turn_radius = ((left_vel + right_vel) * constants.track_width_meters * 0.5) / (right_vel - left_vel)
            turn_angle = (vel * dt) / turn_radius
            robot_to_turn_center = Translation2d(turn_radius, 0).rotateBy(old_pose.rotation().rotateBy(Rotation2d(math.pi / 2)))
            turn_center_to_new_point = (-robot_to_turn_center).rotateBy(Rotation2d(turn_angle))
            turn_translation = robot_to_turn_center + turn_center_to_new_point
            new_position = Pose2d(old_pose.translation().X() + turn_translation.X(),
                                  old_pose.translation().Y() + turn_translation.Y(),
                                  old_pose.rotation().rotateBy(Rotation2d(turn_angle)))

        return new_position
