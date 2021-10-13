from __future__ import annotations

from typing import List

import ctre

from robot_lib.motor import PIDMotor


class _Talon(PIDMotor):
    _motor: ctre.BaseTalon

    def __init__(self, can_id: int):
        self._can_id = can_id

    def get_sensor_position(self) -> float:
        return self._motor.getSelectedSensorPosition(0)

    def get_sensor_velocity(self) -> float:
        return self._motor.getSelectedSensorVelocity(0)

    def set_raw_output(self, x: float):
        self._motor.set(ctre.ControlMode.PercentOutput, x)

    def set_target_position(self, pos: float):
        self._motor.set(ctre.ControlMode.Position, pos)

    def set_target_velocity(self, vel: float):
        self._motor.set(ctre.ControlMode.Velocity, vel)

    def follow(self, master: _Talon):
        self._motor.follow(master._motor)


class TalonFX(_Talon):
    def init(self):
        self._motor = ctre.TalonFX(self._can_id)


class TalonSRX(_Talon):
    def init(self):
        self._motor = ctre.TalonSRX(self._can_id)


class TalonGroup(PIDMotor):
    motors: List[_Talon]

    def __init__(self, *motors: _Talon, leader_idx: int = 0):
        self.motors = list(motors)
        self._leader_idx = leader_idx

    def init(self):
        self.motors[self._leader_idx].init()
        for idx, motor in enumerate(self.motors):
            if idx != self._leader_idx:
                motor.init()
                motor.follow(self.motors[self._leader_idx])

    def set_leader_idx(self, idx: int):
        self._leader_idx = idx
        # self.motors[self._leader_idx].set_raw_output(0)  # Maybe?
        for idx, motor in enumerate(self.motors):
            if idx != self._leader_idx:
                motor.follow(self.motors[self._leader_idx])

    def get_sensor_position(self) -> float:
        return self.motors[self._leader_idx].get_sensor_position()

    def get_sensor_velocity(self) -> float:
        return self.motors[self._leader_idx].get_sensor_velocity()

    def set_raw_output(self, x: float):
        self.motors[self._leader_idx].set_raw_output(x)

    def set_target_position(self, pos: float):
        self.motors[self._leader_idx].set_target_position(pos)

    def set_target_velocity(self, vel: float):
        self.motors[self._leader_idx].set_target_velocity(vel)
