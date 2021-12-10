#!/usr/bin/env python

# Used for testing the serial ports of the Pi

import serial
from time import sleep

ser = serial.Serial ("/dev/serial0", 38400, timeout=1)    #Open port with baud rate
while True:
    ser.write(0x81)    
    received_data = ser.read()              #read serial port
    sleep(0.1)
    data_left = ser.inWaiting()             #check for remaining byte
    received_data += ser.read(data_left)
    print (received_data)                   #print received data
    ser.write(received_data)                #transmit data serially 