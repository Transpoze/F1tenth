#!/usr/bin/env python  
# publishes the position and direction information from MoCap as an Odometry message

import rospy
from qualisys.msg import Subject
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def mocap_to_odom(mocap):
	global pub, last_x, last_y, last_z, last_t
	odom = Odometry()
	odom.header = mocap.header
	odom.child_frame_id = mocap.header.frame_id # velocity is given in the qualisys frame
	
	curr_t = rospy.get_rostime().to_sec() 
	curr_x = mocap.position.x 
	curr_y = mocap.position.y 
	curr_z = mocap.position.z 
	
	if 'last_t' in globals():		
		dt = curr_t - last_t
		odom.twist.twist.linear.x = (curr_x - last_x) / dt
		odom.twist.twist.linear.y = (curr_y - last_y) / dt
		odom.twist.twist.linear.z = (curr_z - last_z) / dt
	
	odom.pose.pose.position = mocap.position
	odom.pose.pose.orientation = mocap.orientation
	
	pub.publish(odom) 
	last_x, last_y, last_z, last_t = curr_x, curr_y, curr_z, curr_t
	
	


rospy.init_node("MoCap_to_Odom")
mocap_name = rospy.get_param("/mocap_name","F1Tenth")

rospy.Subscriber("qualisys/" + mocap_name, Subject, mocap_to_odom)
pub = rospy.Publisher("odom/measurements", Odometry, queue_size=10)

rospy.spin()
