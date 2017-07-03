#! /usr/bin/env python
# Reads gps-data from Emlid Reach unit ad publishes two topics:
# 1) standard sensor_msgs/NavSatFix message
# 2) custom gps_reach message in the NMEA format (see http://www.rtklib.com/prog/manual_2.4.2.pdf page 102)

import rospy
import rospkg
import numpy as np
import serial

from get_gps.msg import gps_reach
from sensor_msgs.msg import NavSatFix

def receiver():
	try:
		port = '/dev/ttyACM0'
		ser = serial.Serial(port=port, baudrate=115200, timeout=1)
		wait = False
	except serial.serialutil.SerialException:
		# load text file with dummy data for testing
		print 'port not found, sending dummy data...'
		wait = True		
		ser = open(rospkg.RosPack().get_path('get_gps')+'/params/gpslog2.txt')
	
	rospy.init_node('gps_receiver', anonymous=True)
	rate = rospy.Rate(10)	

	pub_reach = rospy.Publisher('gps_reach', gps_reach, queue_size=10)
	pub_navsat = rospy.Publisher('gps_navsat', NavSatFix, queue_size=10)

	solution_types = ['fixed', 'float', 'reserved', 'DGPS', 'single']
	
	gps = gps_reach()
	navsat = NavSatFix()
	while not rospy.is_shutdown():
		line = ser.readline()		
		words = line.split()
		
		if len(words) == 15:
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
			
			gps.stn = float(words[7])
			gps.ste = float(words[8])
			gps.stu = float(words[9])
			gps.stne = float(words[9])
			gps.steu = float(words[10])
			gps.stun = float(words[11])

			navsat.position_covariance = [gps.stn,  gps.stne, gps.stun,
				 gps.stne, gps.ste,  gps.steu,
				 gps.stun, gps.steu, gps.stu]
			
			gps.age = float(words[12])
			gps.ratio = float(words[13])

		pub_reach.publish(gps)
		pub_navsat.publish(navsat)
		
		if wait:
			rate.sleep()

receiver()
