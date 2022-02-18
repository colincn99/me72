#!/usr/bin/env python

import RPi.GPIO as GPIO  
from time import sleep 

# Set up laser
GPIO.setmode(GPIO.BCM)
power_pin = 21
GPIO.setup(power_pin, GPIO.OUT)  
  
try:  
    while True:
        # Keep setting laser high
        GPIO.output(power_pin, 1) 
        sleep(1)
  
except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt  
    GPIO.cleanup()                 # resets all GPIO ports used by this program