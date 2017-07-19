#!/usr/bin/env python
'''
Node for PID control.

'''

import rospy
import numpy as np
from slip_control_communications.msg import input_model
from slip_control_communications.msg import input_pid
#yaw-rate parameters
sim_rate = 10
kp_y=0.01 
ki_y =1#1#0.001#0.1#0.01
kd_y = 0.
prev_yaw_rate_error = None
integral_error_y = 0.0
yaw_rate= 0.0

pub = rospy.Publisher('drive_parameters_topic', input_model, queue_size=10)

#velocity parameters
kp_v = 0.01#0
ki_v = 1 #1#0.001#0.01
kd_v = 0.
prev_velocity_error = None
integral_error_v = 0.0
velocity= 0.0



#-------------------------------------------------------------------------------
# control
#
# INPUTS:
#   data: a input_pid type message: velocity and angle error with respect to
#         the reference the vehicle is to track
#-------------------------------------------------------------------------------

def control(data):
    
    global prev_yaw_rate_error
    global integral_error_y
    global kp_y
    global ki_y
    global kd_y
    global yaw_rate
    global kp_v
    global ki_v
    global kd_v
    global integral_error_v
    global prev_velocity_error
    global velocity
  
    #yaw-rate controller

    yaw_rate_error = data.error_yaw_rate

    if prev_yaw_rate_error == None:
        delta_error_y = 0
    else:
        delta_error_y = yaw_rate_error - prev_yaw_rate_error

    prev_yaw_rate_error = yaw_rate_error
    integral_error_y += yaw_rate_error


    

    angle =(kp_y*yaw_rate_error + kd_y*delta_error_y*sim_rate + ki_y*integral_error_y/sim_rate)
  
    while angle > np.pi/3:
        angle = np.pi/3
    while angle < -np.pi/3:
        angle = -np.pi/3
   

  #velocity controller
    velocity_error = data.error_velocity

    if np.abs(velocity_error)<0.1:
        velocity_error=0.1

    if prev_velocity_error == None:
        delta_error_v = 0
    else:
        delta_error_v = velocity_error - prev_velocity_error

    prev_velocity_error = velocity_error
    integral_error_v += velocity_error

    if abs(velocity_error) < 0.01:
        integral_error_v = 0.

    velocity =  (kp_v*velocity_error + kd_v*delta_error_v*sim_rate + ki_v*integral_error_v/sim_rate) #- kd*delta_error*sim_rate

   
    msg = input_model();
    msg.velocity = velocity
    msg.angle =angle
    rospy.loginfo(msg)
    pub.publish(msg)

#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('pid_controller_node', anonymous=True)
    print("Listening to error")

    rospy.Subscriber("error_topic", input_pid, control)
    rospy.spin()
