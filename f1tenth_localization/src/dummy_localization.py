#! /usr/bin/env python

import rospy
import rospkg
import numpy as np
import serial
from sensor_msgs.msg import Imu

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def callback_gps(gps_input):
	eps = 0.01

	odom = Odometry()
	odom.pose.pose.position.x = gps_input.latitude 
	odom.pose.pose.position.y = gps_input.longitude
	
	pub_ekf.publish(odom)
	
	rospy.loginfo('x-position from localization: {}'.format(gps_input.latitude))
	rospy.loginfo('y-position from localization: {}'.format(gps_input.longitude))
	
def nothing(x):
	return
	
	
def dummy_gps():
	global pub_ekf
	
	rospy.init_node('dummy_ekf', anonymous=True)
	rospy.Subscriber('imu', Imu, nothing)
	rospy.Subscriber('gps_navsat', NavSatFix, callback_gps)

	rate = rospy.Rate(10)	
	pub_ekf = rospy.Publisher('car_state_topic', Odometry, queue_size=10)
	
	rospy.spin()

dummy_gps()
