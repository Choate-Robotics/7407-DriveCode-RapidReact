from typing import Callable, List, Type

import commands2 as commands

from robot_lib.subsystem import Subsystem


class Command(commands.CommandBase):
    _REQUIREMENTS: List[Subsystem] = []

    def __init__(self):
        super().__init__()
        self.addRequirements(*self._REQUIREMENTS)


def requires(*subsystems: Subsystem) -> Callable[[Type[Command]], Type[Command]]:
    def decorator(cls: Type[Command]) -> Type[Command]:
        cls._REQUIREMENTS += subsystems
        return cls
    return decorator
