#!/usr/bin/python3

import socket
import signal
import math
from typing import Tuple, List
import pathlib
from alive_progress import alive_bar

HOST = '192.168.2.69'
PORT = 6666

PACKED_POINT_LEN = 7

TICKS_PER_REV = 20390
UP_PITCH = -91
FOWARD_OFFSET_MM = 12

running = True

def sig_handler(sig, frame):
    global running
    running = False
signal.signal(signal.SIGINT, sig_handler)

def packed_point_to_str(byte_arr):
    
    if byte_arr[1] & 0b1 != 1:
        print("bad check bit")
    quality = (byte_arr[0] >> 2)
    angle = ((byte_arr[1] >> 1) + (byte_arr[2] << 7)) / 64.0
    distance = (byte_arr[3] + (byte_arr[4] << 8)) / 4.0
    azimuth_ticks = ((byte_arr[6] << 8) + byte_arr[5])
    return (f"{angle:3.3f},{distance:.1f},{quality},{azimuth_ticks}", azimuth_ticks)

# Capture the points
with open('raw_capture.txt', 'w') as raw_f:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(2)
        sock.connect((HOST, PORT))
        sock.settimeout(10)
        buf = b''
        with alive_bar(title="Rotation Progress", manual=True) as bar:
            while running:
                buf += sock.recv(1)
                if len(buf) == PACKED_POINT_LEN:
                    point_str, ticks = packed_point_to_str(buf)
                    bar(ticks/TICKS_PER_REV)
                    #print(point_str)
                    raw_f.write(point_str + '\n')
                    buf = b''

