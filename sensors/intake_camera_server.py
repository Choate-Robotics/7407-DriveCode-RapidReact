import socket
import struct
import time
from multiprocessing.connection import Client, Listener

import serial
from serial import SerialException

local_conn = Client(('localhost', 6000))
ds_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

port = '/dev/ttyACM0'


def try_connect():
    try:
        return serial.Serial(port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                             xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None,
                             dsrdtr=True)
    except SerialException:
        return None


serial_port = try_connect()


def wait(t):
    s = time.time()
    while time.time() < s + t:
        if local_conn.poll(0.001):
            local_conn.recv()
            local_conn.send(None)


def read_frame():
    global serial_port
    try:
        serial_port.write("snap".encode())
        serial_port.flush()
        size = struct.unpack('<L', serial_port.read(4))[0]
        return serial_port.read(size)
    except SerialException:
        serial_port = None
        return None


while True:
    ds_socket.sendto(b"hello", ('<broadcast>', 5804))
    start = time.time()
    frame = None

    if serial_port is None:
        serial_port = try_connect()
        # wait(3)
    else:
        frame = read_frame()

    end = time.time()
    fps = 1 / (end - start)

    if local_conn.poll(0.0001):
        cmd = local_conn.recv()
        print(f"cmd = {cmd}")
        local_conn.send(fps)
        if frame is not None:
            ds_socket.sendto(frame, ('<broadcast>', 5804))
