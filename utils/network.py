from networktables import NetworkTables, NetworkTable


class Network:
    limelight_table: NetworkTable
    test_table: NetworkTable

    @staticmethod
    def init():
        NetworkTables.initialize("10.74.7.2")
        Network.limelight_table = NetworkTables.getTable("limelight")
        Network.test_table = NetworkTables.getTable("test")
