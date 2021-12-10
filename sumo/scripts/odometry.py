#!/usr/bin/env python
#
#   odometry.py
#
#   Odometry node.
#
#   Node:       /odometry
#   Publish:    /odom                   geometry_msgs/TransJointState
#               TF map -> odom          geometry_msgs/TransformStamped
#               /wheel_command          sensor_msgs/JointState
#   Subscribe:  /vel_cmd                geometry_msgs/Twist
#               /wheel_state            sensor_msgs/JointState
#
import math
import rospy
import tf2_ros

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import JointState


#
#   Constants
#
R = 1 #0.04               # Wheel radius
d = 1 #0.14986 / 2        # Halfwidth between wheels


# Global variables
lpsi_last = 0.0         # Last wheel position (psi)
rpsi_last = 0.0

x = 0.0                 # Current pose (x,y,theta)
y = 0.0
theta = 0.0 


#
#   Twist Command Callback
#
def callback_vel_cmd(msg):
    # Grab the forward and spin velocities.
    vx = msg.linear.x
    wz = msg.angular.z

    # Compute the wheel velocities.
    lpsi_dot = ( vx - d*wz) / R
    rpsi_dot = (-vx - d*wz) / R

    # Create the wheel command msg and publish.
    msg_wcmd = JointState()
    msg_wcmd.header.stamp = rospy.Time.now()
    msg_wcmd.name         = ['M1', 'M2']
    msg_wcmd.velocity     = [lpsi_dot, rpsi_dot]
    pub_wcmd.publish(msg_wcmd)


#
#   Wheel State Callback
#
def callback_wheel_state(msg):
    # Grab the wheel position/velocities.
    try:
        lpsi     = msg.position[msg.name.index('M1')]
        lpsi_dot = msg.velocity[msg.name.index('M1')]
        rpsi     = msg.position[msg.name.index('M2')]
        rpsi_dot = msg.velocity[msg.name.index('M2')]
    except:
        rospy.logerr("Ill-formed /wheel_state message!")
        return

    # Compute the velocity.
    vx = 0.5 * R   * (  lpsi_dot - rpsi_dot)
    wz = 0.5 * R/d * (- lpsi_dot - rpsi_dot)

    # Compute the wheel position changes.
    global lpsi_last, rpsi_last
    lpsi_diff = lpsi - lpsi_last
    rpsi_diff = rpsi - rpsi_last
    lpsi_last = lpsi
    rpsi_last = rpsi

    # Compute the position/heading change
    dp     = 0.5 * R   * (  lpsi_diff - rpsi_diff)
    dtheta = 0.5 * R/d * (- lpsi_diff - rpsi_diff)

    # Update the pose.
    global x, y, theta
    x     += dp * math.cos(theta + 0.5*dtheta)
    y     += dp * math.sin(theta + 0.5*dtheta)
    theta += dtheta

    # Create the odometry msg and publish.
    msg_odom = Odometry()
    msg_odom.header.stamp            = msg.header.stamp
    msg_odom.header.frame_id         = 'odom'
    msg_odom.child_frame_id          = 'base'
    msg_odom.pose.pose.position.x    = x
    msg_odom.pose.pose.position.y    = y
    msg_odom.pose.pose.position.z    = 0.0
    msg_odom.pose.pose.orientation.x = 0.0
    msg_odom.pose.pose.orientation.y = 0.0
    msg_odom.pose.pose.orientation.z = math.sin(theta/2)
    msg_odom.pose.pose.orientation.w = math.cos(theta/2)
    msg_odom.twist.twist.linear.x    = vx
    msg_odom.twist.twist.linear.y    = 0.0
    msg_odom.twist.twist.linear.z    = 0.0
    msg_odom.twist.twist.angular.x   = 0.0
    msg_odom.twist.twist.angular.y   = 0.0
    msg_odom.twist.twist.angular.z   = wz
    pub_odom.publish(msg_odom)

    # Create the transform msg and broadcast.
    msg_tf = TransformStamped()
    msg_tf.header.stamp            = msg.header.stamp
    msg_tf.header.frame_id         = 'odom'
    msg_tf.child_frame_id          = 'base'
    msg_tf.transform.translation.x = x
    msg_tf.transform.translation.y = y
    msg_tf.transform.translation.z = 0.0
    msg_tf.transform.rotation.x    = 0.0
    msg_tf.transform.rotation.y    = 0.0
    msg_tf.transform.rotation.z    = math.sin(theta/2)
    msg_tf.transform.rotation.w    = math.cos(theta/2)
    brd_tf.sendTransform(msg_tf)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('odometry')

    # Create a publisher to send wheel commands.
    pub_wcmd = rospy.Publisher('/wheel_command', JointState, queue_size=10)

    # Create a publisher to send odometry information.
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)

    # Create a TF2 transform broadcaster.
    brd_tf = tf2_ros.TransformBroadcaster()


    # Create a subscriber to listen to twist commands.
    rospy.Subscriber('/vel_cmd', Twist, callback_vel_cmd)

    # Create a subscriber to listen to wheel state.                          
    rospy.Subscriber('/wheel_state', JointState, callback_wheel_state)


    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Odometry spinning...")
    rospy.spin()
    rospy.loginfo("Odometry stopped.")
