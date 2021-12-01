from lib.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain, SwerveNode


class NEOSwerveNode(SwerveNode):
    def init(self):
        pass

    def set_angle_raw(self, pos: float):
        pass

    def set_velocity_raw(self, vel_tw_per_second: float):
        pass


class Drivetrain(SwerveDrivetrain):
    n_00 =
