from typing import Any, Union

import commands2


class Subsystem(commands2.SubsystemBase):
    def init(self): ...

    def get_fields(self) -> dict[str, Union[str, float, int, bool, list[str], list[int], list[float], list[bool]]]:
        return {"name": self.__class__.__name__}
