#!/usr/bin/env python  
# publishes the position and direction information from MoCap as an Odometry message

import rospy
from qualisys.msg import Subject
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def mocap_to_odom(mocap):
	global pub
	odom = Odometry()
	
	odom.header = mocap.header
	odom.header.frame_id = "qualisys"
	
	odom.pose.pose.position = mocap.position
	odom.pose.pose.orientation = mocap.orientation
	
	pub.publish(odom) 
	


rospy.init_node("MoCap_to_Odom")
mocap_name = rospy.get_param("/mocap_name","F1Tenth")

rospy.Subscriber("qualisys/" + mocap_name, Subject, mocap_to_odom)
pub = rospy.Publisher("odom/measurements", Odometry, queue_size=10)

rospy.spin()
