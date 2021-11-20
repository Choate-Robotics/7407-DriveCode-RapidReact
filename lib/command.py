from typing import Generic, TypeVar
import commands2

from lib.subsystem import Subsystem


T = TypeVar("T", bound=Subsystem)


class Command(commands2.CommandBase):
    pass


class SubsystemCommand(Command, Generic[T]):
    def __init__(self, subsystem: T):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(subsystem)
