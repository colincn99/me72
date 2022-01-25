#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf2_ros
import sys
import RPi.GPIO as GPIO
import cv2

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg   import CameraInfo
from sensor_msgs.msg   import Image
from sensor_msgs.msg   import LaserScan
from std_msgs.msg      import Byte

import RPi.GPIO as GPIO  
from time import sleep 

# Set up line laser input pin
GPIO.setmode(GPIO.BCM)
input_pin = 12
GPIO.setup(input_pin, GPIO.IN)

# Set up ROS
rospy.init_node('linesensor')
pub = rospy.Publisher('/line', Byte, queue_size=10)
servo = rospy.Rate(100)
  
try:  
    while not rospy.is_shutdown():
        linemsg = Byte()
        num = 0
        input_read = int(GPIO.input(input_pin))
        num += input_read
        
        # Check if any line sensors have made detections
        if num != 0:
            linemsg.data = num
            print("Sending from linesensor.py")
            pub.publish(num)
        servo.sleep()
  
except KeyboardInterrupt:          # trap a CTRL+C keyboard interrupt
    rospy.loginfo("End convert image to laser scan.")
    GPIO.cleanup()                 # resets all GPIO ports used by this program