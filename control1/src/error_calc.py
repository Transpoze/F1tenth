#!/usr/bin/env python
'''
Node to determine deviation from trajectory.

'''

import rospy
import numpy as np
from slip_control_communications.msg import out_pursuit
from slip_control_communications.msg import input_pid
from slip_control_communications.msg import mocap_data
from nav_msgs.msg import Odometry  
from qualisys.msg import Subject


pub = rospy.Publisher('error_topic', input_pid, queue_size=10)

v_desired=0
y_desired=0
flag=0
prev_x = None
prev_y = None
prev_t = None
prev_yaw = None
sim_rate=10

def callback1(traj):
    global v_desired,y_desired,yaw_rate_desired,flag
    flag=1
    v_desired=traj.velocity_req
    y_desired=traj.yaw_rate_req

def callback2(state):
     global v_desired, y_desired,flag,prev_x,prev_y,prev_t,prev_yaw
     if flag==1:
         x=state.pose.pose.position.x
         y=state.pose.pose.position.y
         yaw=state.pose.pose.orientation.w
         t=state.header.stamp.secs + state.header.stamp.nsecs*1e-9
         #x=state.position.x
         #y=state.position.y
         #yaw=state.orientation.w
         #t=state.header.stamp.secs + state.header.stamp.nsecs*1e-9
         print(repr(x))
         if prev_x == None:
           delta_x = 0
         else:
           delta_x = x - prev_x
         prev_x=x
         if prev_y == None:
            delta_y = 0
         else:
            delta_y = y - prev_y
         prev_y=y
         if prev_t == None:
            delta_t = 0
         else:
            delta_t = t- prev_t
         prev_t=t
         delta_t=delta_t
         velocity=np.sqrt(delta_x*delta_x+delta_y*delta_y)/delta_t
         if prev_yaw==None:
           delta_yaw = 0
         else:
           delta_yaw = yaw - prev_yaw
         prev_yaw=yaw
         yaw_rate=delta_yaw/delta_t
         vel_error=-velocity+ v_desired
      
         yaw_rate_error=-yaw_rate+y_desired
         msg = input_pid()
         msg.yaw_rate=yaw_rate
         msg.velocity=velocity
         msg.error_velocity = vel_error
         msg.error_yaw_rate = yaw_rate_error
         pub.publish(msg)
         rospy.loginfo(msg)

if __name__ == '__main__':
    rospy.init_node('error_calculation_node', anonymous=True)
    print("error_calculation_node")
    rospy.Subscriber("trajectory_request",out_pursuit,callback1)
    #rospy.Subscriber("qualisys/F1Tenth",Subject,callback2)
    rospy.Subscriber("car_state_topic",Odometry , callback2)
    rospy.spin()
