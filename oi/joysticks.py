from typing import List

import wpilib


DRIVER_CONTROLLER_ID = 0
OPERATOR_CONTROLLER_ID = 1


class Joysticks:
    joysticks: list[wpilib.Joystick] = [
        wpilib.Joystick(DRIVER_CONTROLLER_ID),
        wpilib.Joystick(OPERATOR_CONTROLLER_ID)
    ]
