#! /usr/bin/env python

import rospy
import tf
import numpy as np
import utm
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped

rospy.init_node('waypoints_transform')

def waypoints_to_frame(waypoints, frame="odom"):
	global waypoint_pub
	
	# Get the utm->odom transform from navsat_transform_node
	tfbuffer = tf2_ros.Buffer() 
	tflistener = tf2_ros.TransformListener(tfbuffer)
	T = tfbuffer.lookup_transform(frame, 'utm' , rospy.Time())
	
	# Define temporary PoseStamped message for passing to transformPose
	utm_pose = PoseStamped()
	utm_pose.header.frame_id = 'utm'
	utm_pose.header.stamp = waypoints.header.stamp
	for i,pose in enumerate(waypoints.poses):
		# Transform lat/long to utm
		utm_coords = utm.from_latlon(pose.position.y, pose.position.x)
		rospy.loginfo(utm_coords)
		
		# Store utm coordinates in PoseStamped message
		utm_pose.pose.position.x = utm_coords[0]
		utm_pose.pose.position.y = utm_coords[1]
		
		# Transform utm to local coordinates
		local_pose = tf2_geometry_msgs.do_transform_pose(utm_pose, T)
		waypoints.poses[i] = local_pose.pose
	
	waypoints.header.frame_id = frame
	rospy.loginfo(waypoints)
	waypoint_pub.publish(waypoints)

rospy.Subscriber('/gps/waypoints', PoseArray, waypoints_to_frame)
waypoint_pub = rospy.Publisher('/waypoints', PoseArray, queue_size=1)

rospy.spin()