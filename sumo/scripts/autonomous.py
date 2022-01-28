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
START_DELAY   = 1.0   # Keyboard Timeout
CMD_DT = 0.005         # Time between sending commands

VMAX = 20.0           # Max forward speed (m/s)
WMAX = 20.0           # Max angular speed (rad/s)

LASER_THRESH = 0.05    # Proportion of angles needed for detection

# First elements are acted on first
PRIORITY_LIST = ['RAM',       #
                 'EDGE',      #
                 'TRACK_CAM', #
                 'TRACK_IR',  #
                 'TRACK_NONE' #
                 ]
PRIORITY_DICT = {k: v for v, k in enumerate(PRIORITY_LIST)}

N_LINE = 4

#
#   Global Variables
#
LAST_DIFF = 1e-6   # Arbitrary small positive number

CMD_PRIORITY = 1e6 # Arbitrary large number
CMD = []           # Array of commands (v, omega, command end time from start)
CMD_TIME = rospy.Time(0)  # Time when the trajectory command starts

# Curses Interface for modes
RAM = False
TRACK = False

# Used by other functions/sensor to request a command
# Requests will be blocked if the priority is lower than an active command
def request_command(priority, cmd):
    global CMD_PRIORITY, CMD, CMD_TIME
    
    if isinstance(priority, str):
        priority = PRIORITY_DICT[priority]
    now = rospy.Time.now()
    if priority <= CMD_PRIORITY: #allow command through if same more higher priority
        CMD_PRIORITY = priority
        CMD = cmd
        CMD_TIME = now

# Removes parts of the current command that are no longer valid
# Ensures the current command is valid to currently run
# If command is finished, sets priority to infinite
def update_cur_cmd(now):
    global CMD_PRIORITY, CMD
    
    while len(CMD) != 0:
        if now-CMD_TIME < rospy.Duration(CMD[0][2]):
            return
        else:
            CMD.pop(0)
            
    if len(CMD) == 0:
        CMD_PRIORITY = 1e6

#   Execute velocity commands on a timer based on current commands
def callback_timer(event):
    global CMD_PRIORITY, CMD, CMD_LEN, CMD_TIME
    
    now = event.current_real
    update_cur_cmd(now)
    cmdmsg = Twist()
    
    if len(CMD) == 0 or not(RAM or TRACK):
        # Send stop command
        cmdmsg.linear.x  = 0
        cmdmsg.angular.z = 0
    else:
        # Send velocity command
        if CMD_PRIORITY == 0:
            print('Sending RAM')
        else:
            print('Sending TRACK')
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
    l_detect = left_count > LASER_THRESH * mid
    r_detect = right_count > LASER_THRESH * mid

    if l_detect and r_detect and RAM:
        #Move forward if enough hits on both sides
        request_command('RAM', [[VMAX, diff * WMAX / mid, 0.25]])
    elif not l_detect and not r_detect:
        #Track towards sign which previously had the most hits if both under threshold
        request_command('TRACK_CAM', [[0, WMAX * np.sign(LAST_DIFF), 0.06]])
    else:
        #Track towards side with more hits
        request_command('TRACK_CAM', [[0, diff * WMAX / mid, 0.06]])
        LAST_DIFF = diff

def callback_line(linemsg):
    string = "{0:08b}".format(linemsg.data)
    line_bool = [False] * N_LINE
    for i in range(N_LINE):
        line_bool[i] = string[7-i] == '1'
    print(line_bool)
    

def callback_dist(distmsg):
    pass

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
    RAM_TIME = None
    TRACK_TIME = None

    # Run the servo loop until shutdown.
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        keycode = screen.getch()
        if keycode == ord('q'):
            RAM = False
            TRACK = False
            RAM_TIME = None
            TRACK_TIME = None
        
        if keycode == ord('r'):
            RAM_TIME = now
        elif keycode == ord('t'):
            TRACK_TIME = now
        
        if RAM_TIME is not None:
            RAM = now-RAM_TIME > rospy.Duration(START_DELAY)
        if TRACK_TIME is not None:
            TRACK = now-TRACK_TIME > rospy.Duration(START_DELAY) or RAM

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
    
    # Run Terminal loop
    try:
        curses.wrapper(loop)
    except KeyboardInterrupt:
        pass
    
    # Send a stop command
    request_command(0, [[0, 0, 1]])
    rospy.sleep(0.25)
    rospy.loginfo("Autonomy node stopped.")

