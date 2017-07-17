#! /usr/bin/env python

import rospy
import numpy as np

class ekf:
	def __init__(self, lr, lf):
		# x, y, x velocity, y velocity, x acceleration, y acceleration, yaw, yaw rate, steering angle  
		self.state = np.zeros(9)
		self.lr = lr
		self.lf = lf
		
	def predict(delta):
		x = self.state[0]
		y = self.state[1]
		x_dot = self.state[2]
		y_dot = self.state[3]
		x_ddot = self.state[4]
		y_ddot = self.state[5]
		phi = self.state[6]
		phi_dot = self.state[7]
		angle = self.state[8]
		
		
		cp = np.cos(phi)
		sp = np.sin(phi)
		beta = np.arctan()
		
		
		self.state[0] += delta * ((1 + .5* delta ) * cp  - (1 + .5*delta) * sp )