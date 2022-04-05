import pickle
import socket
import struct
import time
from multiprocessing.connection import Client, Listener

import serial
from serial import SerialException

local_conn = Client(('localhost', 6000))

ds_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
ds_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
ds_socket.bind(('', 5804))

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
        frame_pickle = pickle.dumps((fps, None))
        local_conn.send(len(frame_pickle) if frame_pickle is not None else None)
        if frame is not None:
            try:
                ds_socket.sendto(frame_pickle, ('<broadcast>', 5804))
            except ConnectionRefusedError:
                print("Connection refused...")

# import socket
# import threading
# import os
#
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
#
# sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
#
# sock.bind(('',5804))
# def sendMessage(mes):
# 	sock.sendto(bytes(mes,"utf-8"), ('<broadcast>', 5804))
#
# def receieveMessageLoop():
# 	while True:
# 		data, addr = sock.recvfrom(1024)
# 		#print(addr,socket.gethostbyname(socket.gethostname()))
# 		if addr[0] != socket.gethostbyname(socket.gethostname()):
# 			print(data.decode("utf-8"))
#
# def inputLoop():
# 	while True:
# 		mes = input("")
# 		if mes == "quit()":
# 			os._exit(1)
# 		sendMessage(mes)
#
# #receieveMessageLoop()
#
# t1 = threading.Thread(target=receieveMessageLoop)
# t2 = threading.Thread(target=inputLoop)
#
# # starting thread 1
# t1.start()
# # starting thread 2
# t2.start()
#
# # wait until thread 1 is completely executed
# t1.join()
# # wait until thread 2 is completely executed
# t2.join()