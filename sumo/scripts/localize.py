#!/usr/bin/env python
#
#   localize.py
#
#   This is simply a node with structure similar to the localization.
#   I.e. it reads the messages and transforms, as I expect the
#   localization might needs.  And it publishes the map->odom
#   transform. Currently only publishes pose to make rviz work
#   but will have added functionality in the future
#
#   Node:       /localization
#
#   Params:     ~update_fraction    Fraction of udpate to apply
#
#   Subscribe:  /map                nav_msgs/OccupancyGrid
#               /scan               sensor_msgs/LaserScan
#               /odom               nav_msgs/Odometry
#               /initialpose        geometry_msgs/PoseWithCovarianceStamped
#               /linessensor        std_msgs/Int8
#
#   Publish:    /pose               geometry_msgs/PoseStamped
#
#   TF Required:  odom -> laser
#
#   TF Provided:  map -> odom
#
import math
import numpy as np
import rospy
import tf2_ros

from scipy.spatial import cKDTree

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg      import OccupancyGrid
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import LaserScan
from std_msgs.msg   import Int8

from PlanarTransform import PlanarTransform

######################################################################
#   Global variables
######################################################################
# Localization: Odom frame (Odometry starting pose) w.r.t. map
map2odom = PlanarTransform.basic(x = 0.0, y = 0.0, theta = 0.0)

# Map info (width, height, resolution, origin_x, origin_y)
map_info = None
# The map (2D numpy array).
map = None # Occupancy probabilities of each pixel (u/v coordinates)
wall_points = None # List of wall points (u/v coordinates)
wall_points_map = None # List of nearest wall points to every pixel (u/v coordinates)
wall_tree = None

CONTACT_THRESHOLD = 0.3
# CONTACT_THRESHOLD = 1.5
THRESHOLD = 0.03
OCCUPANCY_THRESHOLD = 80
COUNT_THRESHOLD = 50

MIN_SCAN = 0.05
MAX_SCAN = 2

######################################################################
#   Odometry to Pose Message
######################################################################
#   Odometry Callback
#
#   The odometry message provides the robot's base w.r.t. the odom
#   frame.  Here we compute the robot w.r.t. the map frame.
#   I.e. simply concatenate the map->odom and odom->base poses.
#
#   We could skip this entirely.  Just to view the pose with respect
#   to the map in RVIZ.
#
def callback_odom(odommsg):
    # Make sure the odometry is published w.r.t. the odom frame.
    if not (odommsg.header.frame_id == 'odom'):
        rospy.logerr("Odometry is not in 'odom' frame!")
        return

    # Extract the bot's pose (base) w.r.t. the odom frame.
    # Pre-multiplying by the current odom frame to get the base
    # w.r.t. the map frame.
    odom2base = PlanarTransform.fromPose(odommsg.pose.pose)
    map2base  = map2odom * odom2base

    # Create and publish the pose message.  Note the original Software
    # Architecture doc called for PoseWithCovarianceStamped.  But RVIZ
    # display wants a plain PoseStamped.  And we don't have the
    # covariance anyway.
    posemsg = PoseStamped()
    posemsg.header.stamp    = odommsg.header.stamp
    posemsg.header.frame_id = 'map'
    posemsg.pose            = map2base.toPose()
    posepub.publish(posemsg)
    broadcast_odom(odommsg.header.stamp)


######################################################################
#   Initial Pose Message
######################################################################
#   Initial Pose Callback
#
#   This comes from the RVIZ "2D Pose Estimate" button, telling us
#   where we want the robot to be.
def callback_init(posemsg):
    # Make sure the pose is published w.r.t. the map frame.
    if not (posemsg.header.frame_id == 'map'):
        rospy.logerr("Initial Pose is not in 'map' frame!")
        return

    # Extract the bot's pose (base) w.r.t. the map frame.
    map2base = PlanarTransform.fromPose(posemsg.pose.pose)

    # Also grab the base w.r.t. the odom frame, at the same time.
    tfmsg = tfBuffer.lookup_transform('odom', 'base',
                                      posemsg.header.stamp,
                                      rospy.Duration(1.0))
    odom2base = PlanarTransform.fromTransform(tfmsg.transform)

    # Compute the matching odom frame, reseting the localization.
    global map2odom
    map2odom = map2base * odom2base.inv()
    broadcast_odom(posemsg.header.stamp)

    
######################################################################
#   Setup and Utility Routines
######################################################################
#   Broadcast the Map->Odom Transform
#
#   This provides the latest localization, implicitly adjusting the
#   bot's pose and aligning future laser scans to the map.
def broadcast_odom(timestamp):
    # Prepare the transform message.
    tfmsg = TransformStamped()
    tfmsg.header.stamp    = timestamp
    tfmsg.header.frame_id = 'map'
    tfmsg.child_frame_id  = 'odom'
    tfmsg.transform       = map2odom.toTransform()

    # Broadcast.
    tfbroadcast.sendTransform(tfmsg)

    # Report.
    # rospy.loginfo("Broadcast map->odom: %s" % map2odom)


#   Grab the Map
#
#   We could do this repeatedly, i.e. set up a subscriber.  But I'm
#   assuming the map will remain constant?
def grab_map():
    # Report
    rospy.loginfo("Waiting for a map...")

    # Grab a single message of the map topic.  Give it 5 seconds.
    mapmsg = rospy.wait_for_message('/map', OccupancyGrid, 5.0)

    # Note the map info.
    w      = mapmsg.info.width
    h      = mapmsg.info.height
    res    = mapmsg.info.resolution
    origin = PlanarTransform.fromPose(mapmsg.info.origin)

    # Save the map.  From the map message documentation:
    # The map data, in row-major order, starting with (0,0).
    # Occupancy probabilities are in the range [0,100].  Unknown is -1.
    global map
    global map_info
    origin_x = origin.x()
    origin_y = origin.y()
    
    map_info = (w, h, res, origin_x, origin_y)
    map = np.array(mapmsg.data, dtype=np.int8).reshape(h, w)

    # Report.
    rospy.loginfo("Have the map: %dx%d = %5.3fm x %5.3fm (%4.3fm resolution)"
                  % (w, h, w*res, h*res, res))
    rospy.loginfo("Note map origin %s" % origin)


######################################################################
#   Main Callback
######################################################################
#   Laser Scan Callback
#
#   Do the fun stuff!
def callback_int8(scanmsg):
    # Use the global map->odom transform.
    global map2odom
    broadcast_odom(scanmsg.header.stamp)


######################################################################
#   Main code
######################################################################
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('localization')

    # Grab the ROS parameters.
    updatefrac = rospy.get_param('~update_fraction', 0.4)
    # updatefrac = 0.1

    # Grab a single copy the map (rather than subscribing).
    '''grab_map()'''


    tfBuffer = tf2_ros.Buffer()
    tflisten = tf2_ros.TransformListener(tfBuffer)

    # And create a TF2 transform broadcaster.  Give it a little time
    # to connect, then broadcast the initial odom pose.
    tfbroadcast = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.25)
    broadcast_odom(rospy.Time.now())


    # Create a publisher to send the pose messages.
    posepub = rospy.Publisher('/pose', PoseStamped, queue_size=10)

    # Finally create subscribers to listen to odometry, scan, initial
    # pose messages (to last to reset the localization).
    rospy.Subscriber('/odom',        Odometry,                  callback_odom)
    rospy.Subscriber('/linesensor',  Int8,                      callback_int8, queue_size=1)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, callback_init)

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Localization spinning... (update fraction %f)"
                  % updatefrac)
    rospy.spin()
    rospy.loginfo("Localization stopped.")
