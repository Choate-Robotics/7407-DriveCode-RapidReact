from sensors.intake_camera_network import UDPClient

client = UDPClient()

while True:
    print(client.recv_packet())
