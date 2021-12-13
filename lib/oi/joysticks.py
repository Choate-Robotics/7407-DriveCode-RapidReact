from dataclasses import dataclass
from typing import List

import wpilib


DRIVER_CONTROLLER_ID = 0
OPERATOR_CONTROLLER_ID = 1


class Joysticks:
    joysticks: list[wpilib.Joystick] = [
        wpilib.Joystick(DRIVER_CONTROLLER_ID),
        wpilib.Joystick(OPERATOR_CONTROLLER_ID)
    ]


@dataclass
class JoystickAxis:
    controller_id: int
    axis_id: int

    @property
    def value(self) -> float:
        return Joysticks.joysticks[self.controller_id].getRawAxis(self.axis_id)
