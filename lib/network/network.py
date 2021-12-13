from networktables import NetworkTables, NetworkTable

from lib.subsystem import Subsystem
from robot_systems import Robot
from utils import logger


class Network:
    dashboard_data_table: NetworkTable

    subsystems: list[Subsystem]
    subsystem_names: list[str]

    @staticmethod
    def init():
        NetworkTables.initialize("10.74.7.2")
        Network.dashboard_data_table = NetworkTables.getTable("dashboard_data")

        Network.subsystems: list[Subsystem] = list(
            {k: v for k, v in Robot.__dict__.items() if isinstance(v, Subsystem)}.values()
        )

        Network.subsystem_names = []

        for s in Network.subsystems:
            Network.subsystem_names.append(s.__class__.__name__)

        logger.info(f"Creating tables for subsystems: {Network.subsystem_names}", "[network]")

        Network.dashboard_data_table.putStringArray("subsystems", Network.subsystem_names)

    @staticmethod
    def update():
        for s in Network.subsystems:
            subsystem_table = Network.dashboard_data_table.getSubTable(s.__class__.__name__)
            fields = s.get_fields()

            for k, v in fields.items():
                pass
