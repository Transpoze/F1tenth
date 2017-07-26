#!/usr/bin/env python 

import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
import time

def plot_pose(odom):
	global counter
	
	if counter % 10 == 0:
		x = odom.pose.pose.position.x
		y = odom.pose.pose.position.y
		plt.plot(x, y, '*r')
		
		plt.axis("equal")
		plt.draw()
		plt.pause(1e-6)
	
	counter += 1

def plot_waypoints(waypoints):
	for waypoint in waypoints.poses:
		plt.plot(waypoint.position.x, waypoint.position.y, 'bo')
		plt.draw()
		plt.pause(1e-6)
	
	

if __name__ == '__main__':
	counter = 0
	
	rospy.init_node("plotter")
	rospy.Subscriber("/odometry/filtered", Odometry, plot_pose)
	rospy.Subscriber("/waypoints", PoseArray, plot_waypoints)
	
	plt.ion()
	plt.show()
	
	rospy.spin()