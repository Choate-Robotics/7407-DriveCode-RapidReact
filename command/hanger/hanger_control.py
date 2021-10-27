from robot_lib.command import requires, Command
from robot_systems import Robot


@requires(Robot.hanger)
class HangerControl(Command):
    def __init__(self, motor_percent: float):
        super().__init__()
        self._motor_percent = motor_percent

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        Robot.hanger.motor.set_raw_output(self._motor_percent)

    def end(self, interrupted: bool) -> None:
        Robot.hanger.motor.set_raw_output(0)

    def runsWhenDisabled(self) -> bool:
        return False
