#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray, Pose

def goalPublisher():
    pub = rospy.Publisher('waypoints', PoseArray, queue_size=10)
    rospy.init_node('goalPublisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    #while not rospy.is_shutdown():
    point1=Pose()
    point2=Pose()
    point3=Pose()
    point4=Pose()
    point5=Pose()
    Array=PoseArray()
    point1.position.x=10
    point1.position.y=10

    # point2.position.x=0
    # point2.position.y=5
    # point3.position.x=1.144
    # point3.position.y=-1.565
    # point4.position.x=0.624
    # point4.position.y=-2.256
    # point5.position.x=-0.140
    # point5.position.y=-2.490

    Array.poses.append(point1)
    # Array.poses.append(point2)
    # Array.poses.append(point3)
    # Array.poses.append(point4)
    # Array.poses.append(point5)
       
    rospy.loginfo(Array)
    pub.publish(Array)
    rospy.spin()
if __name__ == '__main__':
    try:
        goalPublisher()
    except rospy.ROSInterruptException:
        pass