from typing import Generic, TypeVar
import commands2

from lib.base_subsystem import BaseSubsystem


T = TypeVar("T", bound=BaseSubsystem)


class BaseCommand(commands2.CommandBase):
    pass


class SubsystemCommand(BaseCommand, Generic[T]):
    def __init__(self, subsystem: T):
        super().__init__()
        self.subsystem = subsystem
        self.addRequirements(subsystem)
