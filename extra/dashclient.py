import socket
import sys
import cv2
import pickle
import datetime
import numpy as np
import struct ## new
import zlib
from PIL import Image
import pygame
import zlib

HOST='10.74.7.2'
#HOST=''
PORT=8504

max_fps = 60
img_size = (320, 240)


def convert_buff(buffer):
    try:
        buff = np.asarray(Image.frombuffer("RGB", img_size, buffer, "jpeg", "RGB", ""))
        #print(buff)
    except Exception as e:
        print ("JPEG decode error (%s)"%(e))
        return None
    
    if (buff.size != (img_size[0]*img_size[1]*3)):
        return None
    return (img_size[0], img_size[1], buff.reshape((img_size[1], img_size[0], 3)))




client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#client_socket.connect(('192.168.1.124', 8485))
client_socket.connect((HOST, PORT))
client_socket.settimeout(2)
#client_socket.connect(('127.0.0.1', 8485))
connection = client_socket.makefile('wb')



#conn,addr=client_socket.accept()
data = b""
payload_size = struct.calcsize(">L")


pygame.init()
# init screen
running = True
Clock = pygame.time.Clock()
font = pygame.font.SysFont("monospace", 50,bold=True)
dimensions = (1380, 650)
padding = 30
screen = pygame.display.set_mode(dimensions)
screen.fill((110, 140, 210))
image_scale = 2

print("GOT")
screen.blit(font.render(f"Swag Cameras", 1, (255, 255, 255)), (200, 50))
while True:
    Clock.tick(200)
    while len(data) < payload_size:
        #print("Recv: {}".format(len(data)))
        fromclient = client_socket.recv(4096)
        if fromclient == b'': #socket dead
            sys.exit()
        data += fromclient
    print("Done Recv: {}".format(len(data)))
    packed_msg_size = data[:payload_size]

    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]
    print("msg_size: {}".format(msg_size))
    while len(data) < msg_size:
        data += client_socket.recv(4096)
    raw_data = data[:msg_size]
    data = data[msg_size:] #reset data var
    frame_data = zlib.decompress(raw_data) #unzip data
    (raw,raw2)=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
    fb1 = convert_buff(raw)
    fb2 = convert_buff(raw2)
    fps = Clock.get_fps()
    print(f"FPS: {round(fps,2)}")
    if fb1 != None:
        # create image from RGB888
        image = pygame.image.frombuffer(fb1[2].flat[0:], (fb1[0], fb1[1]), 'RGB')
        image = pygame.transform.scale(image, (img_size[0]*image_scale, img_size[1]*image_scale))
        # screen.blit(image, (dimensions[0] -img_size[0]*image_scale - padding, padding+110))
        # TODO check if res changed
        # blit stuff
        screen.blit(image, (padding, padding+110))
    if fb2 != None:
        # create image from RGB888
        image = pygame.image.frombuffer(fb2[2].flat[0:], (fb2[0], fb2[1]), 'RGB')
        image = pygame.transform.scale(image, (img_size[0]*image_scale, img_size[1]*image_scale))
        screen.blit(image, (dimensions[0] -img_size[0]*image_scale - padding, padding+110))
        #screen.blit(font.render(f"FPS {fps}", 1, (255, 0, 0)), (0, padding - 20))
        # update display

    pygame.display.flip()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
             running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

    
    # frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
    # cv2.imshow('ImageWindow',frame)
    # cv2.waitKey(1)
