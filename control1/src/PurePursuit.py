#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseArray, Pose
from slip_control_communications.msg import input_model
from slip_control_communications.msg import out_pursuit
from qualisys.msg import Subject
import tf.transformations as tf
from control1.msg import Cmd
#from slip_control_communications.msg import mocap_data
from nav_msgs.msg import Odometry  
import numpy as np
detection=0
#pub = rospy.Publisher('trajectory_request', out_pursuit, queue_size=10)
pub=rospy.Publisher('drive_parameters_topic',input_model,queue_size=10)
wp_x=[]
wp_y=[]
wpt_interpolated_x=[]
wpt_interpolated_y=[]
flag=0
velocity_req=10
discrete_step=10
len_wp=0
prev_x = None
prev_y = None
prev_t = None
prev_yaw = None
def callback1(self):
    global wp_x,wp_y,flag,wpt_interpolated_x,wpt_interpolated_y,len_wp,discrete_step
    interval_x=[]
    interval_y=[]
    len_wp=len(self.poses)
    for i in range(0,len_wp):
     wp_x.append(self.poses[i].position.x)
     wp_y.append(self.poses[i].position.y)
    
    for i in range(0,len_wp-1):
        interval_x.append((wp_x[i+1]-wp_x[i])/discrete_step)
        interval_y.append((wp_y[i+1]-wp_y[i])/discrete_step)
    
    for i in range(0,len_wp-1):
        for j in range(0,discrete_step):
          wpt_interpolated_x.append(wp_x[i]+j*interval_x[i])
          wpt_interpolated_y.append(wp_y[i]+j*interval_y[i])

    print('wpt_interpolated_y:',wpt_interpolated_y)
    flag=1

def calculate_velocity(state):
   global v_desired, y_desired,flag,prev_x,prev_y,prev_t,prev_yaw
   x=state.position.x
   y=state.position.y
   #yaw=state.orientation.w
   t=state.header.stamp.secs + state.header.stamp.nsecs*1e-9
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
   return velocity

# def callback3(boolean):
#   global detection
#   if boolean.cmd.flag:
#     detection=1

def callback2(state):
      global flag,wp_x,wp_y,wpt_interpolated_y,wpt_interpolated_x,len_wp,discrete_step,v_desired, y_desired,flag,prev_x,prev_y,prev_t,prev_yaw,velocity_req,detection
      L = 0.1773+0.1477
      velocity=calculate_velocity(state)
      ld_min=2
      ld_max=4
      k_ld=5

      # Look_ahead_points=int(k_ld*velocity)
      # if Look_ahead_points>=ld_max:
      #  Look_ahead_points=ld_max
      # elif Look_ahead_points<=ld_min:
      #  Look_ahead_points=ld_min
      Look_ahead_points=5
      distance=[]
      ref_state=[]
      x=state.position.x
      y=state.position.y
      euler=tf.euler_from_quaternion([state.orientation.x,state.orientation.y,state.orientation.z,state.orientation.w])
      yaw=euler[2]
      print('yaw',yaw)
      #t=state.header.stamp.secs + state.header.stamp.nsecs*1e-9
      #x=state.pose.pose.position.x
      #y=state.pose.pose.position.y
      #yaw=state.pose.pose.orientation.w
      if flag == 1:
        for i in range(0,len(wpt_interpolated_x)):
         distance.append(np.sqrt((x-wpt_interpolated_x[i])*(x-wpt_interpolated_x[i])+(y-wpt_interpolated_y[i])*(y-wpt_interpolated_y[i])))
        array=np.array(distance)
        k=array.argmin()
        #print(k)
        nearest_point=[wpt_interpolated_x[k],wpt_interpolated_y[k]]
        #print('nearest_point',nearest_point)
        
        if (k+Look_ahead_points)<len(wpt_interpolated_x):
         ref_state.append(wpt_interpolated_x[k+Look_ahead_points])
         ref_state.append(wpt_interpolated_y[k+Look_ahead_points])
        else:
         ref_state.append(wpt_interpolated_x[-1])
         ref_state.append(wpt_interpolated_y[-1])

        dx = ref_state[0] - x
        dy = ref_state[1] - y
        yaw_vector=[np.cos(yaw),np.sin(yaw)]
       # lk_vector=[wpt_interpolated_x[k+Look_ahead_points]-x,wpt_interpolated_y[k+Look_ahead_points]-y]
        dot_product=yaw_vector[0]*lk_vector[0]+yaw_vector[1]*lk_vector[1]
        
        
        ang=np.arccos(dot_product/(np.linalg.norm(yaw_vector)*np.linalg.norm(lk_vector)))
        
        
        # if np.abs(ang)>0.4 and velocity_req>1:
        #   velocity_req-=0.5
        # elif np.abs(ang)<0.4 and velocity_req<10:
        #   velocity_req+=0.5
        


        
        velocity_req=8
        y_goal = -np.sin(yaw) * dx + np.cos(yaw) * dy
        distance_to_goal = np.sqrt(dx**2 + dy**2)
        steering_command =np.arctan(L * 2 * y_goal / distance_to_goal ** 2)
        x_goal=dx*np.cos(yaw)+dy*np.sin(yaw)
       # velocity_req=4
       # yaw_rate_req=velocity_req*steering_command/L
        #yaw_rate_req=velocity_req/L*np.tan(steering_command)
        #gamma=2*x_goal/distance_to_goal **2
        #yaw_rate_req=gamma*velocity_req
        if detection==1:
          velocity_req=0

        #if (k==len(wpt_interpolated_x)-1 and distance_to_goal<2):
         # velocity_req=0
        # msg=out_pursuit()
        # msg.=yaw_rate_req
        # msg.velocity_req=velocity_req
        # pub.publish(msg)
        msg=input_model()
        msg.angle=steering_command
        print(velocity_req)#yaw_rate_req
        msg.velocity=velocity_req
        pub.publish(msg)
        
        
    
def callback3(var):
 global detection
 if var.flag=='True':
  detection=1
 elif var.flag=='False':
  detection=0
 print('detection',detection) 
 
def WaypointsListener():
    
    rospy.init_node('WaypointsListener', anonymous=True)
    print("listening to way-points")
    rospy.Subscriber('waypoints', PoseArray, callback1)
    rospy.Subscriber("qualisys/F1Tenth",Subject,callback2)
    rospy.Subscriber('detect_result', Cmd, callback3)
    #rospy.Subscriber("car_state_topic", Odometry, callback2)
    rospy.spin()

if __name__ == '__main__':
    WaypointsListener()
