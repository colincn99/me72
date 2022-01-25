#!/usr/bin/env python3
#
#   cameratolaserscan.py
#
#   Convert an camera image into with line laser to a laser scan.
#
#   Node:       /cameratolaserscan
#
#   Params:     show
#
#   Subscribe:  None
#
#   Publish:    /scan                   sensor_msgs/LaserScan
#

import math
import numpy as np
import rospy
import sys
import RPi.GPIO as GPIO
import cv2

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg   import CameraInfo
from sensor_msgs.msg   import Image
from sensor_msgs.msg   import LaserScan


#
#   Constants
#
TIMEOUT = 5.0           # Timeout for the initialization
RANGE_MINIMUM = 0.0  # minimum range value [m]
RANGE_MAXIMUM = 3.0   # maximum range value [m]

SCAN_TOPIC  = '/scan'

OFFSET_X = 0.0 # X offset from base frame to camera frame
OFFSET_Y = 0.2 # Y Offset from base frame to camera frame
CAMERA_ANGLE = 0.3 # downward angle with horizontal in radians
FOV_X = 1.2 # Maximum horizontal field of view angle in radians
FOV_Y = 1.2 # Maximum vertical field of view angle in radians


######################################################################
#   Main Code
######################################################################
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('cameratolaserscan')
    
    # Create a publisher for the laser scan.
    pub = rospy.Publisher(SCAN_TOPIC, LaserScan, queue_size=10)
    
    # Show images if argument is provided
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) != 2:
        show = False
    else:
        show = argv[1] == 'show'
    
    # Set up the laser
    GPIO.setmode(GPIO.BCM)
    power_pin = 16
    GPIO.setup(power_pin, GPIO.OUT)
    GPIO.output(power_pin, 1)
    
    # Set up the camera
    cam = cv2.VideoCapture(0)
    ret, img = cam.read()
    
    # Evaluate constants
    h_middle = img.shape[0] / 2
    pixel_ratio = h_middle / np.tan(FOV_Y / 2)
    angle_inc = FOV_X / img.shape[1]
    
    
    while not rospy.is_shutdown():
        timestamp = rospy.Time.now()
        
        ret, img = cam.read()
        
        # Set bounds on BGR values and create a mask
        lowerb = np.array([0, 0, 200])
        upperb = np.array([200, 200, 255])
        red_line = cv2.inRange(img, lowerb, upperb)
        
        # Show image if argument provided
        if show:
            cv2.imshow('red', red_line)
            cv2.waitKey(2)
        
        # Formatting for next line
        x = red_line 
        
        # Find the bottommost nonzero element in every column
        h_raw = np.where(np.count_nonzero(x, axis=0)==0, -1, (x.shape[0]-1) - np.argmin(x[::-1,:]==0, axis=0))
        
        # Evaluate range based on height in picture
        h = h_raw - h_middle
        phi = np.arctan(h / pixel_ratio)
        ranges = OFFSET_Y / np.tan(CAMERA_ANGLE + phi)
        ranges[np.where(h_raw == -1)] = -1

        # Create scan msg.
        scanmsg = LaserScan()
        scanmsg.header.stamp    = timestamp
        scanmsg.header.frame_id = 'base'
        scanmsg.angle_min       = -FOV_X/2
        scanmsg.angle_max       =  FOV_X/2
        scanmsg.angle_increment = angle_inc
        scanmsg.time_increment  = 0.0
        scanmsg.scan_time       = 1/30.0
        scanmsg.range_min       = RANGE_MINIMUM     # minimum range value [m]
        scanmsg.range_max       = RANGE_MAXIMUM     # maximum range value [m]
        scanmsg.ranges          = ranges

        # Publish.
        pub.publish(scanmsg)
  
    rospy.loginfo("End convert image to laser scan.")
    GPIO.cleanup()
