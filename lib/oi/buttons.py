from dataclasses import dataclass

import commands2 as commands
import commands2.button

from lib.oi.joysticks import Joysticks


@dataclass
class Button:
    controller_id: int
    def __call__(self) -> commands.button.Button: ...


@dataclass
class DefaultButton(Button):
    button_id: int

    def __call__(self) -> commands.button.Button:
        if self.button_id < 0:
            return commands.button.Button(
                lambda: Joysticks.joysticks[self.controller_id].getRawAxis(-self.button_id) > 0.8
            )
        return commands.button.JoystickButton(Joysticks.joysticks[self.controller_id], self.button_id)


@dataclass
class AxisButton(Button):
    axis_id: int
    range_min: float = -1
    range_max: float = 1

    def __call__(self) -> commands.button.Button:
        return commands.button.Button(
            lambda: self.range_min <= Joysticks.joysticks[self.controller_id].getRawAxis(self.axis_id) <= self.range_max
        )
