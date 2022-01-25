import math
from dataclasses import dataclass

from ctre import ControlMode
from robotpy_toolkit_7407.motors import TalonFX, TalonConfig
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveOdometry, SwerveDrivetrain
from robotpy_toolkit_7407.utils import logger

from oi.keymap import Keymap


TURN_FF = 20818
TURN_kF = 1023 / TURN_FF
TURN_Cruise_Vel = TURN_FF / 2
TURN_kP = 0.0957142857
TURN_kD = 1.5
TURN_CONFIG = TalonConfig(TURN_kP, 0, TURN_kD, TURN_kF, neutral_brake=True)

MOVE_FF = 21273
MOVE_kF = 1023 / MOVE_FF
MOVE_CONFIG = TalonConfig(0, 0, 0, MOVE_kF, neutral_brake=True)


@dataclass
class TalonFXSwerveNode(SwerveNode):
    m_move: TalonFX
    m_turn: TalonFX
    setpoint: float = 0

    __gear_ratio = (2048 / (2 * math.pi)) * 6.55

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()

    def set_angle_raw(self, pos: float):
        self.setpoint = pos
        pos_sensor_units = pos * TalonFXSwerveNode.__gear_ratio
        self.m_turn.set_target_position(pos_sensor_units)

    def set_velocity_raw(self, vel_tw_per_second: float):
        if self.motor_reversed:
            self.m_move.set_raw_output(-vel_tw_per_second / 8)
        else:
            self.m_move.set_raw_output(vel_tw_per_second / 8)

    def get_current_angle_raw(self) -> float:
        # sensor_pos = self.m_turn.get_sensor_position() / TalonFXSwerveNode.__gear_ratio
        # sensor_pos -= math.pi / 2
        # logger.info(f"sensor_pos={sensor_pos}")
        # return sensor_pos
        return self.setpoint

    def get_current_velocity(self) -> float:
        return self.m_move.get_sensor_velocity()


# TODO Add Pigeon IMU
class GyroOdometry(SwerveOdometry):
    def __init__(self):
        # self._gyro = GyroADIS16448()
        pass

    def init(self):
        # self._gyro.reset()
        pass

    def get_robot_angle_degrees(self) -> float:
        # return -self._gyro.angle
        return 0

    def reset_angle(self):
        # self._gyro.reset()
        pass


class Drivetrain(SwerveDrivetrain):
    n_00 = TalonFXSwerveNode(
        TalonFX(7, config=MOVE_CONFIG),
        TalonFX(8, config=TURN_CONFIG)
    )
    n_01 = TalonFXSwerveNode(
        TalonFX(1, config=MOVE_CONFIG),
        TalonFX(2, config=TURN_CONFIG)
    )
    n_10 = TalonFXSwerveNode(
        TalonFX(5, config=MOVE_CONFIG),
        TalonFX(6, config=TURN_CONFIG)
    )
    n_11 = TalonFXSwerveNode(
        TalonFX(3, config=MOVE_CONFIG),
        TalonFX(4, config=TURN_CONFIG)
    )
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    odometry = GyroOdometry()
