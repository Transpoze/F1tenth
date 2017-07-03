#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseArray, Pose
from slip_control_communications.msg import out_pursuit
#from slip_control_communications.msg import mocap_data
from nav_msgs.msg import Odometry  

pub = rospy.Publisher('trajectory_request', out_pursuit, queue_size=10)
#just publishes constant v and yaw_rate requests for now!!!
x_d=0
y_d=0
flag=0
waypt_x=[0]*2
waypt_y=[0]*2
# def callback1(data):
#     global x_d
#     global y_d
#     global flag
#     global waypt_x
#     global waypt_y
#     i=0
#     for i in range(0,2):
#      waypt_x[i]=data.poses[i].position.x
#      print(waypt_x[i])
#      waypt_y[i]=data.poses[i].position.y
#     flag=1

def callback1():
        global flag   
   # if flag == 1:

        yaw_rate=3
        velocity=5
        msg=out_pursuit()
        msg.yaw_rate_req=yaw_rate
        msg.velocity_req=velocity
        rospy.loginfo(msg)
        pub.publish(msg)
        
def callback2():
        global flag   
   # if flag == 1:

        yaw_rate=-7
        velocity=15
        msg=out_pursuit()
        msg.yaw_rate_req=yaw_rate
        msg.velocity_req=velocity
        rospy.loginfo(msg)
        pub.publish(msg)
        
        
        

def WaypointsListener():
    flag=0

    rospy.init_node('WaypointsListener', anonymous=True)

    print("listening to way-points")
    rate=rospy.Rate(10)
    

    # t_end = time.time() +  30
    # while time.time() < t_end:
    #   callback1()

    while not rospy.is_shutdown():
      callback2()
    
    rospy.Subscriber('waypoints', PoseArray, callback1)
    rospy.Subscriber("car_state_topic", Odometry, callback2)


    # spin() simply keeps python from exiting until this node is stopped
     

if __name__ == '__main__':
    WaypointsListener()
