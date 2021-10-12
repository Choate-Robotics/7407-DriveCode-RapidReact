from __future__ import annotations

from typing import List, Type

import commands2
import commands2 as commands

from robot_lib import robot_2

_SUBSYSTEMS: List[Type[commands.SubsystemBase]] = []


class Watcher(type):
    def __init__(cls, name, bases, cls_dict):
        if len(cls.mro()) > 2:
            cls: Subsystem

            class RobotSubsystem(commands2.SubsystemBase):
                def __init__(self) -> None:
                    super().__init__()
                    cls.init()

                def getName(self) -> str:
                    return cls.__name__

                def periodic(self) -> None:
                    cls.periodic()

            RobotSubsystem.__name__ = cls.__name__
            _SUBSYSTEMS.append(RobotSubsystem)

        super(Watcher, cls).__init__(name, bases, cls_dict)


class Subsystem(metaclass=Watcher):
    @staticmethod
    def init() -> None:
        pass

    @staticmethod
    def periodic() -> None:
        pass


class Drivetrain(Subsystem):
    pass

pass
