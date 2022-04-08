import socket
import struct
import time
import pickle
import serial
import zlib
from threading import Thread
rgb = False
ports = ['/dev/ttyACM0', '/dev/ttyACM1']
HOST = ''
PORT = 8504
max_fps = 24
# max_fps = 40

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM,)
print('Socket created')

s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

s.bind((HOST, PORT))
print('Socket bind complete')
s.listen(10)
print('Socket now listening')
connections = []
active_ports = []


def dump(s_port):
    s_port.write("snap".encode())
    s_port.flush()
    # num_bytes = struct.unpack('<L', buffer)[0]
    num_bytes = struct.unpack('<L', s_port.read(4))[0]

    buff = s_port.read(num_bytes)
    return buff


def attemptConnections(ports):
    while True:
        for camera_port in ports:
            if camera_port not in map(lambda a: a.port, active_ports):
                try:
                    new_port = serial.Serial(camera_port, baudrate=115200, bytesize=serial.EIGHTBITS,
                                             parity=serial.PARITY_NONE,
                                             xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None,
                                             dsrdtr=True)
                    print("connected to: " + camera_port)
                    active_ports.append(new_port)
                except Exception as e:
                    print("Failed to init: " + camera_port)
            else:
                print("Camera already initiated")
        time.sleep(5)


frames_buff = None
def dumpBuffers():
    global frames_buff
    while True:
        # Clock.tick(max_fps)
        temp_buff = [b''] * len(ports)
        for i in range(len(active_ports)):
            try:
                temp_buff[i] = dump(active_ports[i])
            except:
                del active_ports[i]
        frames_buff = temp_buff
        #print(len(frames_buff[0]), len(frames_buff[1]))
        #buff_q.put_nowait(frames_buff)
        time.sleep(1/50)


def send_to_clients():
    global frames_buff
    while True:
        #frames_buff = buff_q.get()
        # print(len(frames_buff[0]),len(frames_buff[1]))
        #data = pickle.dumps(frames_buff, 0)
        data = zlib.compress(pickle.dumps(frames_buff, 0))
        size = len(data)
        print(len(struct.pack(">L", size) + data), len(frames_buff[1]))
        # print(size)
        for connection in connections:
            try:
                connection.sendall(struct.pack(">L", size) + data)
            except IOError:
                connections.remove(connection)
                print("client disconnect")
        time.sleep(1/max_fps)

threads = []

cam_conn = Thread(target=attemptConnections, args=[ports])
threads.append(cam_conn)
cam_conn.start()

buff_dumper = Thread(target=dumpBuffers, args=[ ])
threads.append(buff_dumper)
buff_dumper.start()

client_thread = Thread(target=send_to_clients, args=[ ])
threads.append(client_thread)
client_thread.start()

while True:
    conn, addr = s.accept()
    connections.append(conn)
    print(threads)

s.close()