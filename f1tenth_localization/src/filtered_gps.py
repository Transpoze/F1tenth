#! /usr/bin/env python

import rospy
import tf
import rospkg
import numpy as np

from get_gps.msg import gps_reach
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray

# rotate orientation object `ori` with quaternion `q`
def rotate_orientation(ori, q):
	rot_mat = tf.transformations.quaternion_matrix(q)
	pose_rot = rot_mat.dot([ori.x, ori.y, ori.z, ori.w])
	ori.x = pose_rot[0]
	ori.y = pose_rot[1]
	ori.z = pose_rot[2]
	ori.w = pose_rot[3]

# translate position object `pos` with vector `t`
def translate_position(pos, t):
	pos.x += t[0]
	pos.y += t[1]
	pos.z += t[2]

# Transforms position estimate from EKF into global coordinates (lat/long)
def odom_to_utm(odom):
	global utm_pub
	trans,rot = listener.lookupTransform('utm', 'odom', rospy.Time(0))
	translate_position(odom.pose.pose.position, trans)
	rotate_orientation(odom.pose.pose.orientation, rot)
	utm_pub.publish(utm)
	
# Transforms global waypoints from the HMI into local coordinates
def utm_waypoints_to_odom(waypoints):
	global waypoint_pub
	trans,rot = listener.lookupTransform('odom', 'utm', rospy.Time(0))
	for waypoint in waypoints.poses:
		translate_position(waypoint, trans)
	waypoint_pub.publish(waypoints)
	
	
rospy.init_node('odom_to_navsat', anonymous=True)

rospy.Subscriber('/odometry/filtered', Odometry, odom_to_utm)
utm_pub = rospy.Publisher('/gps/filtered', NavSatFix, queue_size=1)

rospy.Subscriber('/gps/waypoints', PoseArray, utm_waypoints_to_odom)
waypoint_pub = rospy.Publisher('/waypoints', NavSatFix, queue_size=1)

tf_listener = tf.TransformlListener()


rospy.spin()