import redis
#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from serial import Serial

r = redis.Redis(host='127.0.0.1',port=6379)
file1 = open("test.txt","w+")

#ser = Serial('/dev/cu.usbmodem141401')

#ser2 = Serial('/dev/cu.usbmodem28153001')
# def baud_rate_test(serial_port, packet = b' '):
#     ser = Serial(serial_port)
#     ser.timeout = 0.5
#     for baudrate in ser.BAUDRATES:
#         if 300 <= baudrate <= 57600:
#             ser.baudrate = baudrate
#             ser.write(packet)
#             resp = ser.readall()
#             if resp == packet:
#                 return baudrate
#     return 'Unknown'

# a = baud_rate_test('/dev/cu.usbmodem141401')
# print(a)

while (True):
    pos = r.get('Haptic_POS')
    #pos = pos.decode().strip('][').split(',')
    file1 = open("test.txt","w+")
    file1.write(pos)
    file1.close()
    #ser.write(pos)
    #ser.write(b'\n')
    #print(pos, " ".join(['1.0'] * 3))
    #x = ser.readline()
    #y = ser2.readline()
    #print(pos, x)
ser.close()
