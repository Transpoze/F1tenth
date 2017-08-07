#!/usr/bin/env python  
# computes a moving average of the gravity measured by the imu
# place the imu still on a table and wait until the average converges, then record the measured value in launch_sensors.launch


import rospy
from sensor_msgs.msg import Imu
import numpy as np

def average_acceleration(imu):
	global avg_g
	eps = 0.001
	
	new_g = np.sqrt(imu.linear_acceleration.x**2 + imu.linear_acceleration.y**2 + imu.linear_acceleration.z**2)
	
	if 'avg_g' in globals():
		avg_g = eps*new_g + (1-eps)*avg_g
	else:
		print "first value"
		avg_g = new_g

	print "average g: {}".format(avg_g)
		

rospy.init_node("measure_gravity")
rospy.Subscriber("/imu", Imu, average_acceleration)

rospy.spin()
