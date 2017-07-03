#!/usr/bin/env python

import rospy
from slip_control_communications.msg import mocap_data
i=0
def talker():
     rospy.init_node('mocap_publisher_node', anonymous=True)
     pub = rospy.Publisher('car_state_topic', mocap_data, queue_size=10)
     rate = rospy.Rate(10) # 10hz
     data = mocap_data()

     while not rospy.is_shutdown():
        global i
        i+=1
        data.ts = i
        data.id = 2
        data.x =  2*i
        data.y = 3*i
        data.z = 5*i
        data.roll = 6
        data.pitch = 7
        data.yaw = 8*i
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()
      

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass