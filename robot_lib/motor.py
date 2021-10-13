class MotorType:
    reversible: bool
    encoder: bool
    pos_control: bool
    vel_control: bool


class Motor:
    def __init__(self, can_id: int, motor_type: MotorType):
        self._can_id = can_id
        self._motor_type = motor_type
