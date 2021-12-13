import commands2

from lib.network.objects import Sendable, SubsystemNetworkObject


class Subsystem(commands2.SubsystemBase, Sendable[SubsystemNetworkObject]):
    def init(self): ...

    def get_network_object(self) -> SubsystemNetworkObject:
        return SubsystemNetworkObject(
            name=self.__class__.__name__,
            motors=[]
        )

    def network_update(self, data: SubsystemNetworkObject): ...

