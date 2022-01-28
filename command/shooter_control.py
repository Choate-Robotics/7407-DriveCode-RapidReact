from robotpy_toolkit_7407 import Command
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.utils.math import clamp

from oi.keymap import Keymap
from subsystem.shooter import Shooter


class ShooterControlCommand(SubsystemCommand[Shooter]):
    def initialize(self):
        pass

    def execute(self):
        self.subsystem.set(self.subsystem.current_speed)

        if Keymap.Shooter.INCREMENT_STICK.value > 0.2:
            self.subsystem.current_speed += self.subsystem.increment

        if Keymap.Shooter.DECREMENT_STICK.value > 0.2:
            self.subsystem.current_speed -= self.subsystem.increment

        self.subsystem.current_speed = clamp(self.subsystem.current_speed, -1, 1)

    def isFinished(self) -> bool:
        return False
