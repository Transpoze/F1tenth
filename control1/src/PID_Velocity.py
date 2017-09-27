#!/usr/bin/env python

#Node computing the error in velocity, to be used in a feedback controller

import rospy
import numpy as np
from slip_control_communications.msg import out_pursuit
from slip_control_communications.msg import input_model
from nav_msgs.msg import Odometry  
import tf.transformations as tf

#create a publisher for publishing the velocity error

pub = rospy.Publisher('drive_parameters_topic', input_model, queue_size=10)


# Initialize values

class variable():
    v_desired = 0
    y_desired = 0
    flag = 0
    prev_velocity_error = None
    integral_error_v = 0.0 
    velocity = 0
    kd_v = 0.3
    ki_v = 1
    kp_v = 0.3
    sim_rate = 10
    velocity_old=0
    flag2=1



#Callback from Purepursuit which requests the velocity profile that needs to be maintained

def callback1(traj):   
    #Define flag to check if we have recieved a velocity request from Purepursuit
    
    variable.flag = 1
    variable.v_desired = traj.velocity_req
    variable.y_desired = traj.angle

#Perform control once every measurement update from EKF

def callback2(state):
     
     #Calculate the error once we obtain the request
     
     if variable.flag == 1 and variable.flag2==1:
        #Feedback from EKF node

         x = state.pose.pose.position.x
         y = state.pose.pose.position.y
         t = state.header.stamp.secs + state.header.stamp.nsecs*1e-9
         orientation = state.pose.pose.orientation
         euler = tf.euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
         yaw = euler[2]
         velocity_x = state.twist.twist.linear.x
         velocity_y = state.twist.twist.linear.y
         velocity = ((velocity_x**2)+(velocity_y**2))**0.5

         #Calculate the velocity error

         velocity_error = -velocity+ variable.v_desired
  
         
         if np.abs(velocity_error)<0.1:
           velocity_error=0.1

         if variable.prev_velocity_error == None:
          delta_error_v = 0
         else:
          delta_error_v = velocity_error - variable.prev_velocity_error

         variable.prev_velocity_error = velocity_error
         variable.integral_error_v += velocity_error

         if abs(velocity_error) < 0.1:
          variable.integral_error_v = 0.

         variable.velocity += variable.kp_v*velocity_error #+ variable.kd_v*delta_error_v*variable.sim_rate + variable.ki_v*variable.integral_error_v 
         
         print('velocity',velocity)
         if variable.velocity_old>variable.velocity and variable.velocity < 8:
          variable.velocity = 8
         if velocity>2:
          variable.velocity=0
         print('to_car',variable.velocity) 

         variable.velocity_old=variable.velocity
         msg = input_model()
         if variable.v_desired == 0:
            print('hi')
            msg.velocity = 0
            msg.angle = variable.y_desired
            pub.publish(msg)
            variable.flag2=0

         msg.velocity = variable.velocity
         msg.angle =variable.y_desired
         pub.publish(msg)
         variable.flag = 0

if __name__ == '__main__':
    rospy.init_node('error_calculation_node', anonymous=True)
    print("PID-Node for Velocity")
    rospy.Subscriber("trajectory_request",out_pursuit,callback1)
    rospy.Subscriber('odometry/filtered',Odometry, callback2)
    
    rospy.spin()
