#!/usr/bin/env python  
import roslib
import rospy
# from qualisys.msg import Subject
from nav_msgs.msg import Odometry

import tf

def broadcast(mocap):
    br = tf.TransformBroadcaster()
    br.sendTransform((mocap.pose.pose.position.x, mocap.pose.pose.position.y, mocap.pose.pose.position.z),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "imu_visualization",
					 "odom")

if __name__ == '__main__':
    rospy.init_node('imu_visualization_tf')
    rospy.Subscriber('/odometry/gps', Odometry, broadcast)
    rospy.spin()