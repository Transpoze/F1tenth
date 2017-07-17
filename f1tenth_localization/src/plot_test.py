#!/usr/bin/env python 

import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time

def plot_x(msg):
	global counter
	
	if counter % 10 == 0:
		stamp = msg.header.stamp
		time = stamp.secs + stamp.nsecs * 1e-9
		plt.plot(time, msg.twist.twist.linear.x, '*r')
		# plt.axis("equal")
		plt.draw()
		plt.pause(0.00000000001)
	
	counter += 1
	
if __name__ == '__main__':
	counter = 0
	
	rospy.init_node("plotter")
	rospy.Subscriber("/odometry/filtered", Odometry, plot_x)
	
	plt.ion()
	plt.show()
	
	# for x in range(2,6):
	# 	plt.plot(x,x,'*')
	# 	plt.draw()
	# 	plt.pause(1)
	
	rospy.spin()