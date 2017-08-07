#!/usr/bin/env python  
# This node subtracts gravitational acceleration from the accelerometer measurement. The orientaiton of the imu is taken into account by rotating the gravity vector
# Make sure to measure g using measure_gravity.py since the imu accelerometer measurements can be significantly different from 9.8

import rospy
import tf
import numpy as np
from sensor_msgs.msg import Imu

def set_timestamp(imu):
	global pub, g

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
