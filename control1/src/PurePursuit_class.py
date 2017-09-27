#!/usr/bin/env python
'''
Pure_Pursuit Node calculates the steering command to follow a set of waypoints
and the velocity profile(to be sent to the PID).The controller switches between 2 modes:
1.The 'normal' mode in which the controller tries to follow way-points
2.The 'obstacle avoidance' mode in which a new waypoint is created from the angle feedback 
from the camera so as to avoid the obstacle.

The present structure allows a
dynamic look-ahead based on the velocity and the velocity profile is constant.

TO DO- 
1.Create a velocity profile based on the the curvature of the 
future road(using Look-ahead angle)
2.Interpolate in a better way-Use a Spline instead of linear Interpolation
'''

import rospy
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose
from slip_control_communications.msg import out_pursuit
from slip_control_communications.msg import input_model
from std_msgs.msg import Float32
import tf.transformations as tf
from control1.msg import Cmd
from nav_msgs.msg import Odometry
import numpy as np


class PurePursuit():
	def __init__(self):
		self.L = 0.1773+0.1477  #L is the length of the vehicle
		self.Look_ahead_dist = 1.4 
		self.Look_ahead_dist_obstacle = 1
		self.nearest_point_ind = 0

		self.wpt_interpolated_x = []
		self.wpt_interpolated_y = []
		self.len_wp = 0 

		self.len_wpt_interp = 0
		self.points_per_meter = 5   #Define the density for interpolation(no.of points per meter)


		self.velocity_req = 8

		
		self.flag_normal = True
		self.direction = 0	  #direction to take for avoidance	
		self.x_obst = 0
		self.y_obst = 0
		self.yaw_obst = 0     #vehicle state when detect obstacle
		

		self.x = 0
		self.y = 0
		self.yaw = 0
		self.reached_food_station = False
		rospy.init_node('WaypointsListener', anonymous=True)
		rospy.Subscriber('waypoints', PoseArray, self.way_point_interp)
		#rospy.Subscriber('odometry/filtered',Odometry, self.pure_pursuit_control)
		#rospy.Subscriber('detect_result', Cmd, self.obstacle_detect)
		rospy.Subscriber('car_state_topic', Odometry,self.pure_pursuit_control)        
		self.pub = rospy.Publisher('drive_parameters_topic', input_model, queue_size=10)
		self.pub_angle = rospy.Publisher('target_angle',Float32,queue_size=1)
		rospy.spin()

		
	
	def way_point_interp(self, wp_data):	
		interval_x = []
		interval_y = []

		wp_x=[]
		wp_y=[]
		
		self.len_wp = len(wp_data.poses)

		for i in range(0,self.len_wp):
			wp_x.append(wp_data.poses[i].position.x)
			wp_y.append(wp_data.poses[i].position.y)

		for ind in range(1, len(wp_x)):
			dist_x = wp_x[ind] - wp_x[ind-1]
			dist_y = wp_y[ind] - wp_y[ind-1]

			dist_temp = ((dist_x)**2 + (dist_y)**2)**0.5
			num_points = int(dist_temp * float(self.points_per_meter))

			for num in range(0, num_points):
				temp_x = wp_x[ind-1] + num * dist_x / num_points
				self.wpt_interpolated_x.append(temp_x)
				temp_y = wp_y[ind-1] + num * dist_y / num_points
				self.wpt_interpolated_y.append(temp_y)  

		self.len_wpt_interp = len(self.wpt_interpolated_x)
		

	def pure_pursuit_control(self, vehicle_state):
		ref_state=[]

		self.x = vehicle_state.pose.pose.position.x
		self.y = vehicle_state.pose.pose.position.y
		
		orientation = vehicle_state.pose.pose.orientation
		euler = tf.euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
		self.yaw = euler[2]

		velocity_x = vehicle_state.twist.twist.linear.x
		velocity_y = vehicle_state.twist.twist.linear.y
		self.velocity = ((velocity_x**2)+(velocity_y**2))**0.5


		min_dist = 1000
		
		if self.len_wp != 0:     #checks if there are Waypoints
			if self.flag_normal == True:   #Flag for entering the Normal Mode
				for i in range(self.nearest_point_ind,min(self.len_wpt_interp, self.nearest_point_ind + 4 * self.points_per_meter)):
					temp_dist = (np.sqrt((self.x-self.wpt_interpolated_x[i])*(self.x-self.wpt_interpolated_x[i])+(self.y-self.wpt_interpolated_y[i])*(self.y-self.wpt_interpolated_y[i])))
					if temp_dist < min_dist:
						min_dist = temp_dist
						min_index = i

				self.nearest_point_ind = min_index

				nearest_point=[self.wpt_interpolated_x[self.nearest_point_ind],self.wpt_interpolated_y[self.nearest_point_ind]]

				#if min_dist > self.Look_ahead_dist:
					#ref_state = nearest_point
				if self.nearest_point_ind + self.Look_ahead_dist*self.points_per_meter < self.len_wpt_interp:
					real_num_points = int(self.Look_ahead_dist*self.points_per_meter)# - dist_points)
					ref_state.append(self.wpt_interpolated_x[self.nearest_point_ind+real_num_points])
					ref_state.append(self.wpt_interpolated_y[self.nearest_point_ind+real_num_points])

					# Check if reached Food_station

					if self.reached_food_station == False:
						if self.nearest_point_ind > 0.2 * self.len_wpt_interp and  self.nearest_point_ind < 0.4 * self.len_wpt_interp:
							dx_food = self.x - self.wpt_interpolated_x[int(0.35 * self.len_wpt_interp)]
							dy_food = self.y - self.wpt_interpolated_y[int(0.35 * self.len_wpt_interp)]
							dist_food = np.sqrt(dx_food**2 + dy_food**2)
							if dist_food <= 2.:
								steering_command = 0
								msg=input_model()
								msg.angle=steering_command
								msg.velocity=0
								self.pub.publish(msg)

								self.reached_food_station =True
								rospy.sleep(10)

				else:
					self.nearest_point_ind = self.len_wpt_interp - 1
					ref_state.append(self.wpt_interpolated_x[-1])
					ref_state.append(self.wpt_interpolated_y[-1])

		  
			else: # Enters the Obstacle Avoidance Mode
				ref_state_vehicle=[self.Look_ahead_dist_obstacle*np.cos(self.direction),self.Look_ahead_dist_obstacle*np.sin(self.direction)]

				ref_state_vehicle=[np.cos(-self.yaw_obst)*ref_state_vehicle[0]-np.sin(self.yaw_obst)*ref_state_vehicle[1],-np.sin(-self.yaw_obst) * ref_state_vehicle[0] + np.cos(-self.yaw_obst) * ref_state_vehicle[1]]
				ref_state=[ref_state_vehicle[0]+self.x_obst,ref_state_vehicle[1]+self.y_obst]
		
			# Common Purepursuit after the refernce-to-go is computed 

			dx=ref_state[0]-self.x
			dy=ref_state[1]-self.y
		
			# Conversion to car's frame

			y_goal = -np.sin(self.yaw) * dx + np.cos(self.yaw) * dy
			x_goal = np.cos(self.yaw) * dx - np.sin(-self.yaw) * dy	
			distance_to_goal = np.sqrt(dx**2 + dy**2)
			
			target_ang = np.arctan(y_goal/x_goal)
			self.pub_angle.publish(target_ang)

			# Compute the steering command
			steering_command =np.arctan(self.L * 2 * y_goal / distance_to_goal ** 2)
			
			# Check if the new Waypoint is reached/Obstacle is avoided and then make then enter the normal mode

			if self.flag_normal == False and distance_to_goal <= 0.9: #Tolerance  
				self.flag_normal = True


			# Check if the last Waypoint is reached and make flag as 0 => Ready for new Waypoints

			dx_end = self.wpt_interpolated_x[-1] - self.x
			dy_end = self.wpt_interpolated_y[-1] - self.y
			distance_to_end = np.sqrt(dx_end**2 + dy_end**2)

			if distance_to_end <= 2. and self.nearest_point_ind == self.len_wpt_interp - 1:
				self.velocity_req=0
				steering_command=0
				self.len_wp = 0
			
			if self.velocity>1.6:
				velocity_cmd = 0
			else:
				velocity_cmd = self.velocity_req

			#Publish the trajectory request  

			msg=input_model()
			msg.angle=steering_command
			msg.velocity=velocity_cmd
			self.pub.publish(msg)  




	def obstacle_detect(self, obst_data):
		if self.flag_normal == True:			 		#Check if normal mode
			if obst_data.flag == 'True':				#If Obstacle is present make variable.flag_normal as False
				self.x_obst = self.x 					#Save the state of the car 
				self.y_obst = self.y
				self.yaw_obst = self.yaw

				self.flag_normal = False
				self.direction = obst_data.turn_angle	#The turning required to avoid obstacle

			elif obst_data.flag == 'False':
				self.flag_normal = True

def WaypointsListener():
	PP = PurePursuit()

if __name__ == '__main__':
	WaypointsListener()

		