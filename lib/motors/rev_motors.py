import rev

from lib.motor import Motor


class SparkMax(Motor):  # TODO Implement PID
    _motor: rev.CANSparkMax

    def __init__(self, can_id: int, brushless: bool = True):
        super().__init__()
        self._can_id = can_id
        self._brushless = brushless

    def init(self):
        self._motor = rev.CANSparkMax(
            self._can_id,
            rev.MotorType.kBrushless if self._brushless else rev.MotorType.kBrushed
        )

    def set_raw_output(self, x: float):
        self._motor.set(x)