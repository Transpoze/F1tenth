#!/usr/bin/env python 

import numpy as np
from matplotlib import pyplot as plt
import rospy
from nav_msgs.msg import Odometry
import time

	
if __name__ == '__main__':
	counter = 0
	
	rospy.init_node("fake_gps")
	odom_pub = rospy.Publisher("/odometry/gps", Odometry, queue_size=10)
	
	rate = rospy.timer.Rate(0.5)
	
	x, y = 0, 0
	eps = 0.04
	dx, dy = 0.8, 0
	while not rospy.is_shutdown():
		x += dx
		y += dy
		
		odom = Odometry()
		odom.header.frame_id = "odom"
		odom.header.stamp = rospy.get_rostime()
		
		odom.pose.pose.position.x = x + eps * np.random.normal() 
		odom.pose.pose.position.y = y + eps * np.random.normal() 
		
		odom_pub.publish(odom)
		rate.sleep()