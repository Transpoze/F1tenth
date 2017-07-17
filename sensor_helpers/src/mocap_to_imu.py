#!/usr/bin/env python  
# publishes the direction information from MoCap as an Imu message

import rospy
from qualisys.msg import Subject
from sensor_msgs.msg import Imu

def mocap_to_odom(mocap):
	global pub
	imu = Imu()
	
	imu.header = mocap.header
	imu.header.frame_id = "base_link"
	
	imu.orientation = mocap.orientation
	
	pub.publish(imu) 


rospy.init_node("MoCap_to_Imu")
mocap_name = rospy.get_param("/mocap_name","F1Tenth")
rospy.Subscriber("qualisys/" + mocap_name, Subject, mocap_to_odom)
pub = rospy.Publisher("imu/measurements", Imu, queue_size=10)

rospy.spin()
