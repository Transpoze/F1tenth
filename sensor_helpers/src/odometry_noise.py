#!/usr/bin/env python
# adds artificial noise to odometry message (to simulate gps data)
  
import rospy
import numpy as np
from qualisys.msg import Subject
from nav_msgs.msg import Odometry

def filter(odom):
	global pub, counter
	
	if counter % 50 == 0:
		add_position_noise(odom.pose)
		
		odom.pose.pose.orientation.x = 1.0
		odom.pose.pose.orientation.y = 0.0
		odom.pose.pose.orientation.z = 1.0
		odom.pose.pose.orientation.w = 0.0
		
		pub.publish(odom) 
		
		
	counter += 1

def add_position_noise(pose):
	eps = 0.5
	pose.pose.position.x += eps * np.random.normal() 
	pose.pose.position.y += eps * np.random.normal() 
	
	pose.covariance = [10*eps**2, 0, 0, 0, 0, 0,
						0, 10*eps**2, 0, 0, 0, 0,
						0, 0, 10*eps**2, 0, 0, 0,
						0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0]

rospy.init_node("Odometry_Noise")
rospy.Subscriber("odom/measurements", Odometry, filter)

pub = rospy.Publisher("odom/noise", Odometry, queue_size=10)
counter = 0

rospy.spin()
