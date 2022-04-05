import time

from sensors.intake_camera_network import UDPServer

server = UDPServer()

while True:
    server.send_packet(b"hello world")
