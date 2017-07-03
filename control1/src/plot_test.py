#!/usr/bin/env python 

import numpy as np
from matplotlib import pyplot as plt
import rospy
from slip_control_communications.msg import input_pid
from qualisys.msg import Subject
from nav_msgs.msg import Odometry 
import time

def plot(msg):
	global counter
	now=rospy.get_rostime()
	plt.plot(msg.pose.pose.position.x,msg.pose.pose.position.y, '*')
	plt.axis("equal")
	plt.draw()
	plt.pause(0.00000000001)
	
if __name__ == '__main__':	
	rospy.init_node("plotter")
	rospy.Subscriber('car_state_topic', Odometry, plot)
	plt.ion()
	plt.show()
	rospy.spin()