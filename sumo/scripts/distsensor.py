#!/usr/bin/env python3

import math
import numpy as np
import rospy
import sys

from std_msgs.msg      import Float32MultiArray

import RPi.GPIO as GPIO

# Import the ADS1x15 module.
import Adafruit_ADS1x15


# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()

# Or create an ADS1015 ADC (12-bit) instance.
#adc = Adafruit_ADS1x15.ADS1015()

# Note you can change the I2C address from its default (0x48), and/or the I2C
# bus by passing in these optional parameters:
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 1

# Set up ROS
rospy.init_node('distsensor')
pub = rospy.Publisher('/dist', Float32MultiArray, queue_size=10)
servo = rospy.Rate(10)

try:
    while not rospy.is_shutdown():
        distmsg = Float32MultiArray()
        values = [0]*4
        for i in range(4):
            # Read the specified ADC channel using the previously set gain value.
            values[i] = adc.read_adc(i, gain=GAIN)
            v = values[i] * (3.3 / 32768.0)
            dist = 16.2537 * v**4 - 129.893 * v**3 + 382.268 * v**2 - 512.611 * v + 301.439
            values[i] = dist
            
        print(values)
        distmsg.data = values
        pub.publish(distmsg)
        servo.sleep()

except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt
    rospy.loginfo("End convert image to laser scan.")
    GPIO.cleanup()                 # resets all GPIO ports used by this program