#!/usr/bin/env python
#
#   autonomous.py
#
#   Run the autonomy node based on sensors
#
#   Node:       /move
#
#   Params:     None
#
#   Subscribe:  /scan                   sensor_msgs/LaserScan
#
#   Publish:    /vel_cmd                geometry_msgs/Twist
#
#   TF Required:  None
#
import math
import numpy as np
import rospy
import sys
import tf2_ros
import curses

from geometry_msgs.msg      import Point
from geometry_msgs.msg      import PoseStamped
from geometry_msgs.msg      import TransformStamped
from geometry_msgs.msg      import Twist
from nav_msgs.msg           import OccupancyGrid
from sensor_msgs.msg        import LaserScan
from visualization_msgs.msg import Marker
from sensor_msgs.msg        import Range
from std_msgs.msg           import Byte
from std_msgs.msg           import Float32MultiArray


from PlanarTransform import PlanarTransform


#   Constants
KEY_TIMEOUT    = 0.35 # Keyboard Timeout
CMD_DT = 0.02         # Time between sending commands

VMAX = 20.0           # Max forward speed (m/s)
WMAX = 20.0           # Max angular speed (rad/s)

LASER_THRESH = 0.2    # Proportion of angles needed for detection

# First elements are acted on first
PRIORITY_LIST = ['RAM', 'TRACK']
PRIORITY_DICT = {k: v for v, k in enumerate(PRIORITY_LIST)}

#
#   Global Variables
#
LAST_DIFF = 1e-6   # Arbitrary small positive number

CMD_PRIORITY = 1e6 # Arbitrary large number
CMD = []           # Array of tuples of commands 
CMD_LEN = []       # Array of command lengths
CMD_TIME = rospy.Time(0)  # Time when the first command in the list starts

# Curses Interface for modes
RAM = False
TRACK = False

# Used by other functions/sensor to request a command
# Requests will be blocked if the priority is lower than an active command
def request_command(priority, cmd, cmd_len):
    global CMD_PRIORITY, CMD, CMD_LEN, CMD_TIME
    
    now = rospy.Time.now()
    if priority > CMD_PRIORITY: #allow command through if same or higher priority
        if len(CMD) >= 2: # Block if multiple commands in queue
            return
        if len(CMD) == 1: # Block if final command still valid
            if now-CMD_TIME < rospy.Duration(CMD_LEN[0]):
                return

    CMD_PRIORITY = priority
    CMD = cmd
    CMD_LEN = cmd_len
    CMD_TIME = now

#   Execute velocity commands on a timer based on current commands
def callback_timer(event):
    global CMD_PRIORITY, CMD, CMD_LEN, CMD_TIME
        
    if len(CMD) == 0 or not(RAM or TRACK): # Do nothing if no command
        cmdmsg = Twist()
        cmdmsg.linear.x  = 0
        cmdmsg.angular.z = 0
        cmdpub.publish(cmdmsg)
        return
    
    now = event.current_real
    # Check if current command is still valid
    if now-CMD_TIME > rospy.Duration(CMD_LEN[0]):
        #Remove command if invalid
        CMD_LEN.pop(0)
        CMD.pop(0)
        if len(CMD) == 0: # Do nothing if no command
            cmdmsg = Twist()
            cmdmsg.linear.x  = 0
            cmdmsg.angular.z = 0
            cmdpub.publish(cmdmsg)
            return
        
        CMD_TIME = now
    
    #Send velocity command
    cmdmsg = Twist()
    cmdmsg.linear.x  = CMD[0][0]
    cmdmsg.angular.z = CMD[0][1]
    cmdpub.publish(cmdmsg)

#   Process the Laser Scan
def callback_scan(scanmsg):
    global LAST_DIFF
    
    # Extract the angles and ranges from the scan information.
    ranges = np.array(scanmsg.ranges)
    #alphas = (scanmsg.angle_min + scanmsg.angle_increment * np.arange(len(ranges)))
    
    #Calculate which side has more laser scan hits
    mid = len(ranges)//2
    left = ranges[0:mid]
    right = ranges[mid:-1]
    left_count = np.count_nonzero(left+1)
    right_count = np.count_nonzero(right+1)
    diff = left_count - right_count

    if left_count > LASER_THRESH * mid and \
        right_count > LASER_THRESH * mid and \
        RAM: #Move forward if enough hits on both sides
            request_command(PRIORITY_DICT['RAM'], [[VMAX,diff * WMAX / mid]], [0.2])
    elif left_count < LASER_THRESH * mid and \
        right_count < LASER_THRESH * mid: #Track towards side with more hits
            request_command(PRIORITY_DICT['TRACK'], [[0,WMAX * np.sign(LAST_DIFF)]], [0.2])
    else: #Track towards sign which previously had the most hits if both under threshold
        request_command(PRIORITY_DICT['TRACK'], [[0,diff * WMAX / mid]], [0.2])
        LAST_DIFF = diff

def callback_line(linemsg):
    print("Getting Line callback")
    string = "{0:08b}".format(linemsg.data)
    print(string)

def callback_dist(distmsg):
    print(distmsg.data)

#
#   Terminal Input Loop
#
def loop(screen):
    global RAM, TRACK
    # Make the getchar() non-blocking and otherwise initialize.
    curses.curs_set(0)
    curses.flushinp()
    screen.nodelay(True)
    screen.erase()
    screen.addstr(0, 0, "Hold r for ramming and t for tracking only")
    Tactive = 0.0
    RAM_TIME = rospy.Time(0)
    TRACK_TIME = rospy.Time(0)

    # Run the servo loop until shutdown.
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        keycode = screen.getch()
        if keycode == ord('q'):
            RAM = False
            TRACK = False
            break
        
        if keycode == ord('r'):
            RAM_TIME = now
        elif keycode == ord('t'):
            TRACK_TIME = now
            
        RAM = now-RAM_TIME < rospy.Duration(KEY_TIMEOUT)
        TRACK = now-TRACK_TIME < rospy.Duration(KEY_TIMEOUT) or RAM

        # Wait for the next turn.
        servo.sleep()


######################################################################
#   Main code
######################################################################
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('autonomous')

    cmdpub = rospy.Publisher('/vel_cmd', Twist, queue_size=1)

    rospy.Subscriber('/scan', LaserScan,   callback_scan, queue_size=1)
    
    rospy.Subscriber('/line', Byte, callback_line, queue_size=1)
    
    rospy.Subscriber('/dist', Float32MultiArray, callback_dist, queue_size=1)
    
    

    # And finally, set up a timer to force publication of the obstacle
    # map and waypoints, for us to view in RVIZ.
    timer = rospy.Timer(rospy.Duration(CMD_DT), callback_timer)

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Autonomy node spinning...")
    
    # Setup curses gui
    servo = rospy.Rate(100)
    dt    = servo.sleep_dur.to_sec()
    try:
        curses.wrapper(loop)
    except KeyboardInterrupt:
        pass
    
    # Send a stop command
    request_command(0, [[0,0]], [1])
    rospy.sleep(0.25)
    rospy.loginfo("Autonomy node stopped.")

