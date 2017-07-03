#!/usr/bin/env python

import rospy
import numpy as np
from slip_control_communications.msg import input_model
from nav_msgs.msg import Odometry  
Lf=0.1773
Lr=0.1477
delta_t=0.1
state_x=0
state_y=0
state_yaw=0
prev_x = None
prev_y = None
flag=0
steering_angle=0
v=0
pub = rospy.Publisher('car_state_topic', Odometry, queue_size=10)

def car():
	
		global state_x,state_y,state_yaw,prev_x,prev_y,flag,Lr,Lf,steering_angle,v

		#constraints on the steering angle and velocity achievable
		
		if steering_angle > np.pi/3:
			steering_angle=np.pi/3
		elif steering_angle < -np.pi/3:
			steering_angle=-np.pi/3

		if ((v>-11.3)and(v<11.3)):
		 	v=0

		if v > 100:
			v=100
		elif v < -100:
			v=-100

		#dynamic equations

		beta=np.arctan(Lr/(Lf+Lr)*np.tan(steering_angle))
		state_x=state_x+v*np.cos(state_yaw+beta)*delta_t
		state_y=state_y+v*np.sin(state_yaw+beta)*delta_t
		state_yaw=(state_yaw+v/Lr*np.sin(beta)*delta_t)
			
def callback(control):
	global v,steering_angle
	v=control.velocity
	steering_angle=control.angle
	


def init():

	global state_x,state_y,state_yaw,flag
	rospy.init_node('talker', anonymous=True)
	rospy.Subscriber('drive_parameters_topic',input_model, callback)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
			#print('hi')
			car()
			now=rospy.get_rostime()
			msg=Odometry()
			msg.header.frame_id='map'
			msg.header.stamp.secs=now.secs
			msg.header.stamp.nsecs=now.nsecs
			msg.pose.pose.position.x=state_x
			msg.pose.pose.position.y=state_y
			msg.pose.pose.orientation.w=state_yaw
			pub.publish(msg)
			rospy.loginfo(msg)
			rate.sleep()

if __name__ == '__main__':
    init()

   