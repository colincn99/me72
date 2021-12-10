#!/usr/bin/env python

# Test script for scanning with the line laser and camera
# Made obsolete by cameratolaserscan.py

import RPi.GPIO as GPIO            # import RPi.GPIO module  
from time import sleep             # lets us have a delay
import cv2
import numpy as np
 
GPIO.setmode(GPIO.BCM)             # choose BCM or BOARD
power_pin = 16
GPIO.setup(power_pin, GPIO.OUT)
cam = cv2.VideoCapture(0)
  
try:   
    GPIO.output(power_pin, 1)
    
    while True:
        ret, img = cam.read()
        lowerb = np.array([0, 0, 120])
        upperb = np.array([100, 100, 255])
        red_line = cv2.inRange(img, lowerb, upperb)

        cv2.imshow('red', red_line)
        cv2.waitKey(2)
        x = red_line
        y = np.where(np.count_nonzero(x, axis=0)==0, -1, (x.shape[0]-1) - np.argmin(x[::-1,:]==0, axis=0))
        print(y)
    GPIO.cleanup()
  
except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt  
    GPIO.cleanup()                 # resets all GPIO ports used by this program