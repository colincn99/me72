#!/usr/bin/env python
#
#   wheelcontrol.py
#
#   Controls the wheels and stops if no commands are received after a timeout
#   Plan to interface with encoders in the future
#
#   Node:       /wheelcontrol
#   Publish:    /wheel_state            sensor_msgs/JointState
#   Subscribe:  /wheel_command          sensor_msgs/JointState
#
#   Other Inputs:   Two Encoder Channels (GPIO)
#   Other Outputs:  Motor Driver Commands (via I2C, two channels)
#
import math
import sys
import time
import rospy
import numpy as np
from roboclaw import Roboclaw
from sensor_msgs.msg import JointState


#
#   Constants
#
LEFT  = 0
RIGHT = 1
AXES  = 2
AXISNAME = ['M1', 'M2']
MOTOR_CHANNEL = [0, 1]          # Channel number.  Should be unique.
CMD_MIN = [0, 0]
CMD_PER_RADSEC = [1, 1]

# Tuning parameters and time constants.
RATE         = 50.0
DT           = 1/RATE


T_TIMEOUT    = 0.25     # Shutoff timeout

MINIMUM_SENSE_VEL    = 1.0      # Direction should be clear above this
MINIMUM_MOVE_VEL     = 0.02     # Minimum velocity we want to move
MAXIMUM_FEEDBACK_VEL = 4.0      # Max speed to add for feedback




# Incoming command messages:
cmdvel  = [0] * AXES
cmdtime = [rospy.Time(0)] * AXES


def motor_command(axis, speed):
    '''
    Args:
        axis::int
            Indicates the number of the axis to control
        val::double
            value sent to the axis from -1 to 1 with negative backwards and positive forwards
    Returns:
        None
    '''
    
    cmd_val = min(int(np.abs(speed) * CMD_PER_RADSEC[axis] + CMD_MIN[axis]), 255)
    
    if AXISNAME[axis] == 'M1':
        if speed == 0:
            roboclaw.ForwardM1(address, 0)    
        if speed > 0:
            roboclaw.ForwardM1(address, int(cmd_val))
        else:
            roboclaw.BackwardM1(address, int(cmd_val))
    if AXISNAME[axis] == 'M2':
        if speed == 0:
            roboclaw.ForwardM2(address, 0)
        if speed > 0:
            roboclaw.ForwardM2(address, int(cmd_val))
        else:
            roboclaw.BackwardM2(address, int(cmd_val))
            
#
#   Command Callback Function
#
#   Note this allows for the incoming message to only specify one of
#   the motors (in case we ever wanted to command them independently).
#
def callback_command(msg):
    # Check the message structure.
    if len(msg.name) != len(msg.velocity):
        rospy.logerr("Wheel command msg name/velocity must have same length")
        return

    # Note the current time (to timeout the command).
    now = rospy.Time.now()

    # Extract the velocity commands.
    global cmdvel, cmdtime
    for i in range(len(msg.name)):
        if msg.name[i] in AXISNAME:
            axis = AXISNAME.index(msg.name[i])
            cmdvel[axis]  = msg.velocity[i]
            cmdtime[axis] = now
        else:
            rospy.logerr("Wheel Command msg unknown name '%s'" % msg.name[i])
            return

#
#   Timer Callback Function
#
def callback_timer(event):
    # write to the global variables
    global encdirection, enccount, enclast
    global cmdvel, cmdtime
    global pos_des, vel_des, vel_fbk
    global vel_exp
    global pos, vel, eff

    # Process the commands.
    for axis in range(AXES):
        # Check the validity of the new command.
        if ((event.current_real-cmdtime[axis] > rospy.Duration(T_TIMEOUT))
            or (abs(cmdvel[axis]) < MINIMUM_MOVE_VEL)):
            cmdvel[axis] = 0.0

    # Send the PWM.
    for axis in range(AXES):
        # Set the motor speed from the desired and feedback velocity.
        speed = cmdvel[axis]
        motor_command(axis, speed)
        
        
def callback_encoder(event):

    # Collect the current info into a message and publish.
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name         = AXISNAME
    msg.position     = [0, 0]
    msg.velocity     = [0, 0]
    msg.effort       = [0, 0]
    pub.publish(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('wheelcontrol')

    # Initialize a connection to roboclaw.
    address = 0x80
    roboclaw = Roboclaw("/dev/serial0", 38400)
    roboclaw.Open()
    
    # Check connection
    version = roboclaw.ReadVersion(address)
    print(version)
    if version == (0,0):
        rospy.logerr("Motor Driver not connected!")
    else:
        rospy.loginfo("Motor driver initialized.")

    # Create a subscriber to listen to wheel commands.
    sub = rospy.Subscriber("/wheel_command", JointState, callback_command)

    # Create a publisher to send the wheel state.
    pub = rospy.Publisher('/wheel_state', JointState, queue_size=10)

    # Create the timer.
    timer = rospy.Timer(rospy.Duration(DT), callback_timer)
    
    timer_encoder = rospy.Timer(rospy.Duration(DT), callback_encoder)

    # Spin while the callbacks are doing all the work.
    rospy.loginfo("Running with dt = %.3f sec..." % DT)
    rospy.spin()
    rospy.loginfo("Stopping...")
    
    # Stop the timer (if not already done).
    timer.shutdown()
    
    # Stop motors
    for axis in range(AXES):
        # Set the motor speed from the desired and feedback velocity.
        speed = cmdvel[axis]
        motor_command(axis, 0)



