#!/usr/bin/env python  

import rospy
import tf
import numpy as np
from sensor_msgs.msg import Imu

def set_timestamp(imu):
	global pub, g
	
	# imu.header.stamp = rospy.get_rostime()
	# cov = list(imu.linear_acceleration_covariance)
	# cov[-3] = -1.0
	# imu.linear_acceleration_covariance = tuple(cov)
	
	# Remove gravitational acceleration (put gravity measured by "measure_gravity.py" here!)
	q = [imu.orientation.x,
		imu.orientation.y,
		imu.orientation.z,
		imu.orientation.w]
	
	g_rotated = np.linalg.inv(tf.transformations.quaternion_matrix(q)[:3,:3]).dot(g)
	imu.linear_acceleration.x -= g_rotated[0]
	imu.linear_acceleration.y -= g_rotated[1]
	imu.linear_acceleration.z -= g_rotated[2]
	
	pub.publish(imu) 
		

rospy.init_node("timestamps")
g = [0,0,rospy.get_param('~g',9.8)]
rospy.loginfo("Calibrating accelerometer with g={}".format(g[2]))

rospy.Subscriber("/imu", Imu, set_timestamp)
pub = rospy.Publisher("imu/measurements", Imu, queue_size=10)

rospy.spin()
