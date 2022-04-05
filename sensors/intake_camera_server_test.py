import time

from sensors.intake_camera_network import UDPServer

server = UDPServer()

while True:
    time.sleep(1)
    server.send_packet(b"hello world")
