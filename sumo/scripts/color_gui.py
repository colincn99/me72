#!/usr/bin/env python

import RPi.GPIO as GPIO  
from time import sleep
import cv2
import numpy as np
import Tkinter as tk
import sys

# Get command line argument
if len(sys.argv) == 2:
    mode = sys.argv[1]
else:
    mode = 'raw'

# Set up laser and camera
GPIO.setmode(GPIO.BCM) 
power_pin = 16
GPIO.setup(power_pin, GPIO.OUT)
GPIO.output(power_pin, 1)
cam = cv2.VideoCapture(0)

  
def my_mainloop():
    ret, img = cam.read()

    if mode == 'HSV':
        #Filter image by Hue Saturation Value
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # lower boundary RED color range values; Hue (0 - 10)
        lower1 = np.array([0, l2.get(), l3.get()])
        upper1 = np.array([20, u2.get(), u3.get()])
         
        # upper boundary RED color range values; Hue (160 - 180)
        lower2 = np.array([150,l2.get(),l3.get()])
        upper2 = np.array([179,u2.get(),u3.get()])
        
        lower_mask = cv2.inRange(img, lower1, upper1)
        upper_mask = cv2.inRange(img, lower2, upper2)
        full_mask = lower_mask + upper_mask;
     
        full_mask = lower_mask + upper_mask;
        cv2.imshow('HSV', full_mask)
        cv2.waitKey(2)
    elif mode == 'BGR':
        # Show image filtered by Blue Green Red Values
        lowerb = np.array([l1.get(), l2.get(), l3.get()])
        upperb = np.array([u1.get(), u2.get(), u3.get()])
        red_line = cv2.inRange(img, lowerb, upperb)
        
        cv2.imshow('BGR', red_line)
        cv2.waitKey(2)
    else:
        # Show raw image
        cv2.imshow('img', img)
        cv2.waitKey(2)
        
    master.after(1000, my_mainloop)    

try:
    # Initialize GUI
    master = tk.Tk()
    l1 = tk.Scale(master, from_=0, to=255)
    l1.grid(row=1, column=1)
    l2 = tk.Scale(master, from_=0, to=255)
    l2.grid(row=1, column=2)
    l3 = tk.Scale(master, from_=0, to=255)
    l3.grid(row=1, column=3)
    u1 = tk.Scale(master, from_=0, to=255)
    u1.grid(row=2, column=1)
    u2 = tk.Scale(master, from_=0, to=255)
    u2.grid(row=2, column=2)
    u3 = tk.Scale(master, from_=0, to=255)
    u3.grid(row=2, column=3)

    master.after(1000, my_mainloop)
    master.mainloop()
  
except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt  
    GPIO.cleanup()                 # resets all GPIO ports used by this program
