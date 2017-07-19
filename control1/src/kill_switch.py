#!/usr/bin/env python
'''
Node for emergency stop.
'Del' key for STOP and 'Home' key for NORMAL.

'''

# import rospy
# from std_msgs.msg import Bool
# import curses

# stdscr = curses.initscr()
# curses.cbreak()
# stdscr.keypad(1)
# rospy.init_node('kill_switch_node', anonymous=True)
# em_pub = rospy.Publisher('eStop', Bool, queue_size=10)

# stdscr.refresh()


# key = ''
# while key != ord('q'):
#     key = stdscr.getch()
#     stdscr.refresh()
#     if key == curses.KEY_DC:
#         em_pub.publish(True)
#         stdscr.addstr(5, 20, "Emergency STOP!!!!!")
        
#     elif key == curses.KEY_HOME:
#         em_pub.publish(False)
#         stdscr.addstr(5, 20, "Normal Operation :)")
# rospy.spin()     

#curses.endwin()



#PRESS 2 TO STOP CAR
import pygame, sys
from pygame.locals import *
import rospy
from std_msgs.msg import Bool
rospy.init_node('serial_transmitter_node', anonymous=True)

pygame.init()
pygame.display.set_mode((100,100))
em_pub = rospy.Publisher('eStoper', Bool, queue_size=1)


while True:
   for event in pygame.event.get():
      if event.type == QUIT: sys.exit()
      if event.type == KEYDOWN and event.dict['key'] == 50:
         em_pub.publish(True)
         rospy.loginfo(em_pub)
         pygame.quit()
         pygame.display.quit()
         sys.exit()
   pygame.event.pump()