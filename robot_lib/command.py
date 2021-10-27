from typing import Callable, List, Type

import commands2 as commands

from robot_lib.subsystem import Subsystem
from utils import logger


class Command(commands.CommandBase):
    _REQUIREMENTS: list[Subsystem] = []

    def __init__(self):
        super().__init__()
        self.addRequirements(*self._REQUIREMENTS)

    def as_timed(self, duration: float) -> commands.ParallelDeadlineGroup:
        return commands.ParallelDeadlineGroup(commands.WaitCommand(duration), self)


def requires(*subsystems: Subsystem) -> Callable[[Type[Command]], Type[Command]]:
    def decorator(cls: Type[Command]) -> Type[Command]:
        setattr(cls, "_REQUIREMENTS", subsystems)
        return cls
    return decorator
