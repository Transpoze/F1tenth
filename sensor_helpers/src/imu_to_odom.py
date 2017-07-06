#!/usr/bin/env python  
# converts Imu message to Odometry (for visualizing with rviz)

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def imu_to_odom(imu):
	global pub

	odom = Odometry()
	odom.header = imu.header
	# imu.header.frame_id = "F1Tenth"
	odom.pose.pose.orientation = imu.orientation
	
	pub.publish(odom) 
		
	
rospy.init_node("imu_to_odom")
rospy.Subscriber("/imu", Imu, imu_to_odom)

pub = rospy.Publisher("imu/odometry", Odometry, queue_size=1)

rospy.spin()
