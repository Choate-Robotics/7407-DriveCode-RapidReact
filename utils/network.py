from networktables import NetworkTables, NetworkTable


class Network:
    limelight_table: NetworkTable

    @staticmethod
    def init():
        NetworkTables.initialize("10.74.7.2")
        limelight_table = NetworkTables.getTable("limelight")
