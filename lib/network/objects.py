from typing import TypeVar, Generic

from pydantic import BaseModel


class NetworkObject(BaseModel):
    def serialize(self):
        return self.json()

    @classmethod
    def load(cls, data: str):
        return cls.parse_raw(data)


TObj = TypeVar("TObj", bound=NetworkObject)


class Sendable(Generic[TObj]):
    def get_network_object(self) -> TObj: ...
    def network_update(self, data: TObj): ...


class MotorNetworkObject(BaseModel):
    name: str


class SubsystemNetworkObject(BaseModel):
    name: str
    motors: list[MotorNetworkObject]


class RobotStatusPacket(NetworkObject):
    subsystems: list[SubsystemNetworkObject]
