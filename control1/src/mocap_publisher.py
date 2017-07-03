#!/usr/bin/env python
'''
Node to receive data from Qualisys.

'''

import rospy
import tf
from mocap_source_2 import Mocap, Body
from slip_control_communications.msg import mocap_data

mocap = Mocap(host='SML', info=1)
truck_id = mocap.get_id_from_name("F1TenthB")

#-------------------------------------------------------------------------------
# talker
#-------------------------------------------------------------------------------
def talker():
    rospy.init_node('mocap_publisher_node', anonymous=True)
    pub = rospy.Publisher('car_state_topic', mocap_data, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    data = mocap_data()

    while not rospy.is_shutdown():
        truck_state = mocap.get_body(truck_id)

        if truck_state == 'off':
            rospy.logwarn("Hardware not found!")
        else:
            data.ts = truck_state['ts']
            data.id = truck_state['id']
            data.x = truck_state['x']
            data.y = truck_state['y']
            data.z = truck_state['z']
            data.roll = truck_state['roll']
            data.pitch = truck_state['pitch']
            data.yaw = truck_state['yaw']

        br = tf.TransformBroadcaster()
        br.sendTransform((data.x, data.y, 0),
             tf.transformations.quaternion_from_euler(0, 0, data.yaw),
             rospy.Time.now(),
             "map",
             "base_frame")

        pub.publish(data)
        rate.sleep()


#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
