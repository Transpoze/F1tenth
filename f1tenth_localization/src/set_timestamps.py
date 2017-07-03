#!/usr/bin/env python  

import rospy
from sensor_msgs.msg import Imu

def set_timestamp(imu):
	global pub
	
	imu.header.stamp = rospy.get_rostime()
	imu.linear_acceleration.z = 0
	
	pub.publish(imu) 
		
	
rospy.init_node("timestamps")
rospy.Subscriber("/imu", Imu, set_timestamp)

pub = rospy.Publisher("imu/measurements", Imu, queue_size=10)

rospy.spin()
