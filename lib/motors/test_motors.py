from lib.motor import PIDMotor
from utils.math import clamp


class TestMotor(PIDMotor):
    class ControlMode:
        PERCENT = 0
        POSITION = 1
        VELOCITY = 2

    control_mode: float
    set_point: float
    current_pos: float
    prev_pos: float
    prev_dt: float

    def __init__(self, max_vel: float = 1000):
        self.max_vel = max_vel

    def init(self):
        self.control_mode = self.ControlMode.PERCENT
        self.set_point = 0
        self.current_pos = 0
        self.prev_pos = 0
        self.prev_dt = 0

    def update(self, dt=0.02):
        self.prev_pos = self.current_pos
        self.prev_dt = dt
        if self.control_mode == self.ControlMode.PERCENT:
            self.current_pos += self.set_point * self.max_vel * dt
        if self.control_mode == self.ControlMode.POSITION:
            diff = self.set_point - self.current_pos
            self.current_pos += clamp(diff, -self.max_vel * dt, self.max_vel * dt)
        if self.control_mode == self.ControlMode.VELOCITY:
            self.current_pos += self.set_point * dt

    def set_raw_output(self, x: float):
        self.control_mode = self.ControlMode.PERCENT
        self.set_point = x

    def set_target_position(self, pos: float):
        self.control_mode = self.ControlMode.POSITION
        self.set_point = pos

    def set_target_velocity(self, vel: float):
        self.control_mode = self.ControlMode.VELOCITY
        self.set_point = vel

    def get_sensor_position(self) -> float:
        return self.current_pos

    def get_sensor_velocity(self) -> float:
        if self.control_mode == self.ControlMode.VELOCITY:
            return self.set_point
        if self.prev_dt == 0:
            return 0
        return (self.current_pos - self.prev_pos) / self.prev_dt
