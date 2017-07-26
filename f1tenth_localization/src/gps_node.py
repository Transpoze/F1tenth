#! /usr/bin/env python

import rospy
import rospkg
import numpy as np
import serial

from get_gps.msg import gps_reach
from sensor_msgs.msg import NavSatFix

def receiver():
	try:
		port = rospy.get_param('~port','/dev/ttyACM1')
		ser = serial.Serial(port=port, baudrate=115200, timeout=10)
		wait = False
		rospy.loginfo('Connected to port ' + port)
	except:
		raise Exception('Error connecting to port' + port)
		
		# load text file with dummy data for testing
		# rospy.loginfo('Error connecting to port ' + port + ', sending dummy data...')
		# wait = True		
		# ser = open(rospkg.RosPack().get_path('get_gps')+'/params/gpslog2.txt')
	
	pub_reach = rospy.Publisher('gps/reach', gps_reach, queue_size=10)
	pub_navsat = rospy.Publisher('gps/navsat', NavSatFix, queue_size=10)

	solution_types = ['fixed', 'float', 'reserved', 'DGPS', 'single']
	
	gps = gps_reach()
	navsat = NavSatFix()
	navsat.header.frame_id = "base_link"
	while not rospy.is_shutdown():
		line = ser.readline()		
		
		if line != '':
			words = line.split()
		
			rospy.loginfo("New data from reach: {}".format(line))
		
			try:
				gps.date = words[0]
				gps.time = words[1]
				
				gps.latitude = float(words[2])
				gps.longitude = float(words[3])
				gps.height = float(words[4])
	
				navsat.latitude = float(words[2])
				navsat.longitude = float(words[3])
				navsat.altitude = float(words[4])
				
				gps.solution = solution_types[int(words[5])-1] 
				gps.num_satelites = int(words[6])
				
				gps.stn = 1e-3 * float(words[7])
			
				gps.ste = 1e-3 * float(words[8])
				gps.stu = 1e-3 * float(words[9])
				gps.stne = 1e-3 * float(words[9])
				gps.steu = 1e-3 * float(words[10])
				gps.stun = 1e-3 * float(words[11])

				navsat.position_covariance = [gps.stn,  gps.stne, gps.stun,
				 gps.stne, gps.ste,  gps.steu,
				 gps.stun, gps.steu, gps.stu]
			
				gps.age = float(words[12])
				gps.ratio = float(words[13])
				
				navsat.header.stamp = rospy.get_rostime()
				pub_reach.publish(gps)
				pub_navsat.publish(navsat)
		
			except:
				rospy.loginfo("Error reading GPS data from reach")
				# navsat.header.stamp = rospy.get_rostime()
				# pub_navsat.publish(navsat)
					
				# wait if reading from text file
				# if wait:
				# 	rate.sleep()


rospy.init_node('gps_receiver', anonymous=True)
rate = rospy.Rate(10)	

receiver()

