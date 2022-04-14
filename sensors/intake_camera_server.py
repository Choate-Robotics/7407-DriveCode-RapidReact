import socket
import struct
import time
import pickle
import serial
import zlib
from threading import Thread
import detect_balls
import numpy as np
import pygame
import cv2

rgb = False
ports = ['/dev/ttyACM0', '/dev/ttyACM1']
#/dev/tty.usbmodem3051395331301
HOST = ''
PORT = 8510
max_fps = 24
img_size = (320, 240)



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




def convert_buff(buffer):
    try:
        #buff = np.asarray(Image.frombuffer("RGB", img_size, buffer, "jpeg", "RGB", ""))
        nparr = np.frombuffer(buffer, np.uint8)
        buff = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        #buff = cv2.cvtColor(buff,cv2.COLOR_BGR2RGB)
        #print(buff)
    except Exception as e:
        print ("JPEG decode error (%s)"%(e))
        return None
    
    if (buff.size != (img_size[0]*img_size[1]*3)):
        return None
    return (img_size[0], img_size[1], buff.reshape((img_size[1], img_size[0], 3)))


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
        temp_buff = [b''] * len(ports)
        for i in range(len(active_ports)):
            try:
                temp_buff[i] = dump(active_ports[i])
            except:
                try:
                    del active_ports[i]
                except:
                    pass
        frames_buff = temp_buff
        #print(len(frames_buff[0]), len(frames_buff[1]))
        time.sleep(1/30)


def send_to_clients():
    global frames_buff
    while True:
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

image_scale = 2
def detect_ball_loop():
    global frames_buff
    while True:
        lr_balls = [[],[]]
        (raw,raw2) = frames_buff
        fb1 = convert_buff(raw2)
        fb2 = convert_buff(raw)
        if fb1 != None:
            image = pygame.image.frombuffer(fb1[2].flat[0:], (fb1[0], fb1[1]), 'BGR')
            image = pygame.transform.scale(image, (img_size[0]*image_scale, img_size[1]*image_scale))
            image = detect_balls.pygame_to_cvimage(image)
            [balls,image] = detect_balls.generate_circles(image)
            #image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
            lr_balls[0] = balls
        if fb2 != None:
            image = pygame.image.frombuffer(fb1[2].flat[0:], (fb1[0], fb1[1]), 'BGR')
            image = pygame.transform.scale(image, (img_size[0]*image_scale, img_size[1]*image_scale))
            image = detect_balls.pygame_to_cvimage(image)
            [balls,image] = detect_balls.generate_circles(image)
            #image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
            lr_balls[1] = balls


        #SEND SID lr_balls
        time.sleep(1/30)









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

ball_detector = Thread(target=detect_ball_loop, args=[ ])
threads.append(ball_detector)
ball_detector.start()

while True:
    conn, addr = s.accept()
    connections.append(conn)
    print(threads)

s.close()