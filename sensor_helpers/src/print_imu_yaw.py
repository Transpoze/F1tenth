#!/usr/bin/env python  
# plots the yaw from an imu message. This can be used to debug or to check that the imu is callibrated properly. 

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from qualisys.msg import Subject
import tf
import numpy as np

def get_offset(imu):
	global mocap_orientation
	quaternion = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
	imu_orientation = tf.transformations.euler_from_quaternion(quaternion)
	
	mocap_yaw = mocap_orientation[2] * 180/np.pi
	imu_yaw = imu_orientation[2] * 180/np.pi
	offset = mocap_yaw - imu_yaw
	
	# rospy.loginfo("mocap_yaw: {}".format(mocap_yaw))
	rospy.loginfo("imu_yaw: {}".format(imu_yaw))
	# rospy.loginfo("offset: {}".format(offset))

def get_mocap_data(mocap):
	global mocap_orientaion
	quaternion = [mocap.orientation.x, mocap.orientation.y, mocap.orientation.z, mocap.orientation.w]
	mocap_orientaion = tf.transformations.euler_from_quaternion(quaternion)
	
	
mocap_orientation = [0,0,0]
rospy.init_node("imu_calibration")

mocap_name = rospy.get_param("/mocap_name","F1Tenth")

rospy.Subscriber("/imu", Imu, get_offset)
rospy.Subscriber("/qualisys/" + mocap_name, Subject, get_mocap_data)

rospy.spin()
