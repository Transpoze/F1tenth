#! /usr/bin/env python
# -*- coding: utf-8 -*-

# server_gui.py


'''SML SUMMER PROJECT 2017

AUTONOMOUS CIRCULATION OF F1TENTH CAR
HUMAN MACHINE INTERFACE 
AUTHOR : AUDREY COMEMALE

IMPORTANT : THE BASIS OF THIS CODE HAS BEEN TAKEN FROM AN EXISTING SCRIPT
FROM LABOMEDIA
SEE BELLOW '''

#############################################################################
# Copyright (C) Labomedia February 2015
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franproplin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#############################################################################

#The interface offers two different maps corresponding to the places where the vehicle will run : 
#a parc of KTH Stockholm's campus and and one in Kista 
#The vehicle is represented by a blue square and a black arrow. The data (position, orientation and velocity of the car),
#is sent in real time by the car to the interface through ROS nodes. 
#The latitude and longitude of the maps' borders have to be known to convert the GPS position into the pixel 
#coordinate system. 

#The interface uses Pygame. The origin of the coordinates system is set at the top left corner.
#The window's size is (1571, 872). 
#The interface's board uses the whole length of the window and
#but one part of its width : x goes from 0 to 500. It is composed of texts, variable texts and buttons.
#The map uses the  whole length of the window the its width goes from 500 to 1572. 
#It is possible to zoom in and zoom out thanks to the mousebuttons. For each map, a two zooms are available :
#one for the north part and one for the south part. A real zoom is not possible with Pygame that is why
#it has been faked here by displaying (partie a mentionner dans le rapport technique : trouver une solution inventive
#avec les moyens du bord) the corresponding parts of the map from a closer viewpoint.

#Two classes are defined in the following script : one for buttons and one for the interface, named 'Game'.
#Some methods of the interface are defined within the class while some actions call functions outside the class.

#The window can be shut only with the button QUIT. 

#The interface runs thanks to a while loop where some variables are defined as well as some arbitrary starting points 
#for Kista and KTH. These starting points should be entered by the user or automaticcaly detected by the interface at the beginning of 
#a session. Then, a for loop detects users interaction with the interface by collecting mousebutton or keyboard actions and refreshing buttons
#and texts. Finally, the data received by the subscribers defined in the beginning of the script (talk about global variables) is treated. 
#First, the starting point coordinates are converted form latitude and longitude to pixels. 
#The position received is relative to the starting point. 
#So then, the relative position is converted from meters to pixels. However, for the moment it is multiplied by 2 in order to fit with the simulation
#used to test the code.
#Finally, the position is converted so that the point is in the right place when zooming. The conversion uses affine transformation which has been calculated
#according to the zooms' pictures' sizes and position in the map (detailler un peu plus dans la documentation pour pouvoir être adapte)
#The coodinates are affected to the points according to the map currently displayed. Conditions are set on x and y : they correspond to the borders of the 
#zooms' picture in the map. Outside of they boudaries, the convertion would give coordinates out of the window.

#Pygame physolophy is based on surfaces. A picture such as the map is a surface. The blue square representing the car is also a surface which is pasted on the
#map. Here comes the issue of the screen('s) refreshing. After this conversion part the displaying commands of the maps are called again, outside of the for loop this time. 
#The call in the for loop is useful to give a new reference of map (KTH, Kista, zoom in or zoom out). Once this reference is defined, the blue sqaure is pasted on the map 
#each time a new position is received from ROS nodes. Therefore, the map is quickly full of blue squares correspondong to the former positions of the car.
#To avoid this phenomonon, the screen has to be refreshed by pasting the map at each loop so that one blue square is displayed at a time. 

#Then, text update, status update, velocity calculation, and waypoints display.s

#Talk about the outside functions.

#orientation with respect the abscis axis
#coordinate system
#talk about variables

from __future__ import division
import pygame
import sys
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3, PoseArray, Pose
from turtlesim.msg import Pose as Pos
import datetime
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from pygame.locals import *
from math import sqrt
import numpy as np
from control1.msg import Cmd
from sensor_helper.msg import gps_reach
import tf
#from mocap_source_2 import Mocap, Body
#mocap = Mocap(host='Inet-PC', info=1)
#truck_id = mocap.get_id_from_name("F1tenth_1")


pygame.init()
clock = pygame.time.Clock()

BLACK = 0, 0, 0
WHITE = 255, 255, 255
CIEL = 0, 200, 255
RED = 185, 18, 27
ORANGE = 255, 100, 0
GREEN = 0, 255, 0
GREEN2 = 31, 178, 61
LIGHTBLUE = 202, 237, 233
MARRON = 135, 51, 51
MARRON2 = 189, 141, 70
BEIGE = 246, 228, 151
YELLOW = 252, 250, 225


'''Initialize ROS node'''
rospy.init_node('GUIControl', anonymous=True)

'''Initialize global variables'''
V = None
X = None
Y = None
L_car = None
l_car = None
ori = None
Click = True
GPSQuali = 'Rien'
L = []
Obstacle = 'Nothing'
M = []

'''For velocity calculation, intialization '''
StockPosition = [[0,0],[0,0]]


'''Convert orientation received from nodes'''
def constrainAngle(x):
    return x % 2 * np.pi

def convertRad2Deg(x):
    return(180 * x) / np.pi


'''Define the subscriber to the position'''
def callbackCar_State(msg):
    global L_car, l_car, ori
    #rospy.loginfo("Received a simulation message !!")
    #rospy.loginfo("Position en rospy : [%f, %f]" %(msg.pose.pose.position.x, msg.pose.pose.position.y) )
    #L_car = msg.pose.pose.position.x
    #l_car = msg.pose.pose.position.y
    L_car = msg.longitude
    l_car = msg.latitude
    #ori = msg.pose.pose.orientation.w
    #pose.twist.twist.linear.x/y/z take the magnitude
    #ori = constrainAngle(ori)
    #ori = - convertRad2Deg(ori)
    #rospy.loginfo("recu orientation: [%f]" %(ori))
    #rospy.loginfo("recu position x: [%f, %f]" %(state_x, state_y))


SubsCar_State = rospy.Subscriber("/gps/filtered", NavSatFix, callbackCar_State)
#SubsCar_State = rospy.Subscriber("/car_state_topic", Odometry, callbackCar_State)
#SubsCar_State=rospy.Subscriber("qualisys/F1Tenth", Subject, callbackCar_State)

def callbackData(odom):
    global V, ori
    vel_x = odom.twist.twist.linear.x
    vel_y = odom.twist.twist.linear.y
    V = sqrt((vel_x **2) + (vel_y ** 2))

    q = odom.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    ori = (euler[2]*180 / np.pi) #ori is the yaw of the car
    rospy.loginfo("recu orientation: {}".format(ori ))


SubsData=rospy.Subscriber("odometry/filtered", Odometry, callbackData)

'''Define the subscriber to the GPS Status'''
def callbackGPSQuality(msg):
    global GPSQuali
    GPSQuali = msg.solution
    rospy.loginfo("recu GPS string messages [%s]" %(GPSQuali))

SubsGPSQuality = rospy.Subscriber("/gps/reach", gps_reach, callbackGPSQuality)

'''Define the subscriber to the Obstacle detection'''
def callbackObst_Detect(msg):
    global Obstacle
    #truck_state = mocap.get_body(truck_id)
    #if truck_state != 'off' :
    #    print(type(truck_state), truck_state)
    #    rospy.loginfo(truck_state['x'])
    #    rospy.loginfo(truck_state['y'])

    Obstacle = msg.flag
    rospy.loginfo("received Obst Detect message [%s]" %(msg.flag))

SubsObst_Detect = rospy.Subscriber("/detect_result", Cmd, callbackObst_Detect)



'''Initialize publisher of the waypoints'''
goalPublisher = rospy.Publisher('/gps/waypoints', PoseArray, queue_size = 10)

''' Initialize the publisher to /eStop '''
EmergencyStopPublisher = rospy.Publisher('/eStop', Bool, queue_size = 10)

'''Date''' 
date = datetime.datetime.now()


def rot_center(image, angle):
    orig_rect = image.get_rect()
    rot_image = pygame.transform.rotate(image, angle)
    rot_rect = orig_rect.copy()
    rot_rect.center = rot_image.get_rect().center
    rot_image = rot_image.subsurface(rot_rect).copy()
    return rot_image

class Button:
    '''Add a button with text
    Áction if click
    Black text
    '''

    def __init__(self, fond, text, color, font, dx, dy):
        self.fond = fond
        self.text = text
        self.color = color
        self.font = font
        self.dec = dx, dy
        self.state = False  # enable or not
        self.title = self.font.render(self.text, True, BLACK)
        textpos = self.title.get_rect()
        textpos.centerx = self.fond.get_rect().centerx + self.dec[0]
        textpos.centery = self.dec[1]
        self.textpos = [textpos[0], textpos[1], textpos[2], textpos[3]]
        self.rect = pygame.draw.rect(self.fond, self.color, self.textpos)
        self.fond.blit(self.title, self.textpos)
        


    def update_button(self, fond, action=None):
        self.fond = fond
        mouse_xy = pygame.mouse.get_pos()
        over = self.rect.collidepoint(mouse_xy)
        if over:
            action()
            if self.color == RED:
                self.color = GREEN
                self.state = True
            elif self.color == GREEN:
                # sauf les + et -, pour que ce soit toujours vert
                if len(self.text) > 5:  # 5 char avec les espaces
                    self.color = RED
                self.state = False
        # à la bonne couleur
        self.rect = pygame.draw.rect(self.fond, self.color, self.textpos)
        self.fond.blit(self.title, self.textpos)

    def display_button(self, fond):
        self.fond = fond
        self.rect = pygame.draw.rect(self.fond, self.color, self.textpos)
        self.fond.blit(self.title, self.textpos)


class Game:
    def __init__(self, *args):
        self.screen = pygame.display.set_mode((1542, 872))
        self.level = 0.0 
        self.loop = True
        self.position_perso = (1100,500)
        self.position_arrow = (615,615)
        self.status = 'Waiting to begin'
        self.scale = 1

        ''' Font definition'''
        self.big = pygame.font.SysFont('freesans', 48)
        self.small = pygame.font.SysFont('freesans', 36)
        self.mediumsmall = pygame.font.SysFont('freesans',28)
        self.verysmall = pygame.font.SysFont('freesans', 10)

        self.map1 = 'KistaMap' '''defaultValue'''
        self.VarMess = '...'
        self.create_fond()
        self.create_button()
        self.zoom = 1  #1 if zoom out, 2 if zoom in

        ''' Detecteurs '''
        self.obt_detec = 'Nothing'

    def update_textes(self):
        self.textes = [ ["Autonomous Driving with F1/10", ORANGE, self.small, 0, 50],
                        ["Speed (m/s) :", BLACK, self.small, -95, 150],
                        [str(self.level), BLACK, self.small, 60, 150],
                        ["Status :", BLACK, self.small, -140, 210],
                        [str(self.status), BLACK, self.small, 60, 210],
                        ["GPS quality :", BLACK, self.small, -98, 270],
                        ["Maps :", BLACK, self.small, -145, 400],
                        ["OpenStreetMap ", BLACK, self.verysmall, 142, 750],
                        [str(date), BLACK, self.verysmall, 172, 761],
                        ["Obstacle detection :", BLACK, self.small, -44, 320],
                        [str(self.obt_detec), BLACK, self.small, 175, 320],
                        [str(self.VarMess), BLACK, self.small, 0, 360],
                        ]

    def create_fond(self):
        ''' Background '''
        self.fond = pygame.Surface((500, 872))
        ''' Color '''
        self.fond.fill(WHITE)

    def create_button(self):
        self.reset_button = Button(self.fond, "   Reset   ", YELLOW, self.small, -40, 853)
        self.quit_button  = Button(self.fond, "   Quit   ", RED, self.small, -180, 853)
        self.waypointsStart_button = Button(self.fond, "  Send waypoints  ", MARRON2, self.small, -20, 550)
        self.waypointsReset_button=Button(self.fond,"  Reset waypoints  ", MARRON2, self.small, -16,650)
        self.Kista_button = Button(self.fond, "  Kista  ", BEIGE, self.small, 130, 400)
        self.KTH_button = Button(self.fond, "  KTH  ", BEIGE, self.small, 10, 400)
        self.start_button = Button(self.fond, "  Start  ", GREEN2, self.small, 90, 853 )
        self.stop_button = Button(self.fond, " Stop ", RED, self.small, 200, 853)

    def display_text(self, text, color, font, dx, dy):
        '''Add text on the background. dx gap, dy gap in relation to the center.
        '''
        mytext = font.render(text, True, color)  # True for antialiasing
        textpos = mytext.get_rect()
        textpos.centerx = self.fond.get_rect().centerx + dx
        textpos.centery = dy
        self.fond.blit(mytext, textpos)


    def reset(self):
        global M
        ''' Display the map (KTH or Kista) to erase drawings or pictures. Maps are displayed with a zoom out (self.zoom=1)'''


        KistaMap = pygame.image.load('mapKista.png').convert()
        KTHMap = pygame.image.load('Maskinparken.png').convert()
        if self.map1 == 'KTHE' or self.map1 == 'KTHW' or self.map1 == 'KTHMap' :
            pygame.draw.polygon(KTHMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
            pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000, 840], 2)
            pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
            pygame.draw.line(KTHMap, BLACK, [1000, 840], [1000, 837], 2)
            self.screen.blit(KTHMap, (500,0))
            pygame.display.update()
            self.map1 = 'KTHMap'
        elif self.map1 == 'KistaN' or self.map1 == 'KistaS' or self.map1 == 'KistaMap' :
            pygame.draw.polygon(KistaMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
            pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000, 840], 2)
            pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
            pygame.draw.line(KistaMap, BLACK, [1000, 840], [1000, 837], 2)
            self.screen.blit(KistaMap, (500,0))
            pygame.display.update()
            self.map1 = 'KistaMap'
        self.zoom = 1


        ''' Storage chart : empty '''
        M = []
        L = []
        print("reset")

    def Display_KistaMap(self):
        'Action of "Kista" button. Display Kista s map with zoom out'
        self.zoom = 1
        global Click
        Click = True
        KistaMap=pygame.image.load('mapKista.png').convert()
        pygame.draw.polygon(KistaMap,BLACK, ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
        pygame.draw.polygon(KistaMap,BLACK, ((1030,815),(1032,815),(1032,805), (1030,805),(1027,811),(1027, 811), (1028,807)))
        pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000, 840], 2)
        pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
        pygame.draw.line(KistaMap, BLACK, [1000, 840], [1000, 837], 2)
        self.screen.blit(KistaMap, (500,0))
        self.map1 = 'KistaMap'
        pygame.display.update()

    def Display_KTHMap(self):
        'Action of "KTH" button. Display KTH s map with zoom out'
        self.zoom = 1
        global Click
        Click = True
        KTHMap = pygame.image.load('Maskinparken.png').convert()
        KTHMap = pygame.transform.scale(pygame.image.load('mapKTH.png').convert(), (1042,872))
        pygame.draw.polygon(KTHMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
        pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000, 840], 2)
        pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
        pygame.draw.line(KTHMap, BLACK, [1000, 840], [1000, 837], 2)
        self.screen.blit(KTHMap, (500,0))
        self.map1 = 'KTHMap'
        pygame.display.update()

    def waypoints_personalized(self):
        '''Publishes the waypoints coming stored in M
        Assumes that the max number of waypoints is 10. There is an undetermined number of waypoints which are treated with the list Stock '''
        'Maskinparken'
        LongitMin_KTH = 18.06407
        LongitMax_KTH = 18.06744
        LatitMin_KTH = 59.35174
        LatitMax_KTH = 59.35317
        fenetreWidth = 1042
        fenetreHeight = 872
        EchelleLongit_KTH = ((LongitMax_KTH - LongitMin_KTH) / fenetreWidth)
        EchelleLatit_KTH = ((LatitMax_KTH - LatitMin_KTH) / fenetreHeight)

        '''KistaMap'''
        LongitMin_Kista = 17.95024
        LongitMax_Kista = 17.95360
        LatitMin_Kista = 59.40371
        LatitMax_Kista = 59.40515
        fenetreWidth = 1042
        fenetreHeight = 872
        EchelleLongit_Kista = ((LongitMax_Kista - LongitMin_Kista) / fenetreWidth)
        EchelleLatit_Kista = ((LatitMax_Kista - LatitMin_Kista) / fenetreHeight)

        N = []
        for i in range(len(M)) :
            ''' Affects latitude and longitude distinguishing the cases '''
            if self.map1 == 'KistaMap' or self.map1 == 'KistaS' or self.map1 == 'KistaN' :
                L, l = (M[i][0]-500)*EchelleLongit_Kista + LongitMin_Kista, LatitMax_Kista - M[i][1] * EchelleLatit_Kista
            
            if self.map1 == 'KTHMap' or self.map1 == 'KTHE' or self.map1 == 'KTHW' :
                L, l = (M[i][0]-500)*EchelleLongit_KTH + LongitMin_KTH, LatitMax_KTH - M[i][1] * EchelleLatit_KTH

            N.extend([[L,l]])


        print('Waypoints Personalized')
        #print(Tableau2)
        NbrePoints = len(N)
        point1, point2, point3, point4, point5, point6, point7, point8, point9, point10 = Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose()
        Array = PoseArray()
        Stock = [point1, point2, point3, point4, point5, point6, point7, point8, point9, point10]

        for i in range (NbrePoints):
            Stock[i].position.x, Stock[i].position.y = N[i][0], N[i][1]
        
        Array.poses.extend((Stock[i]) for i in range(NbrePoints))
        Array.header.stamp = rospy.get_rostime()
        Array.header.frame_id = 'utm'
        
        rospy.loginfo(Array)
        goalPublisher.publish(Array)


        
    def WaypointsZoom(self):
        ''' Convert the position in pixel of the waypoints stored in M into the position in pixels adapted to the zooms, stored in L '''
        global L
        L = []

        if self.map1 == 'KTHW' :
            a1, b1 = 1042 / 486, -604
            c1, d1 = 872 / 404, -28
            for i in range(len(M)):
                x, y = M[i][0], M[i][1]
                if x < 1002 and x > 514 and y < 418 and y > 12 :
                    x, y = int(a1 * x + b1), int(c1 * y + d1)
                    L.extend([[x, y]])
                    print('L', L)

        if self.map1 == 'KTHE' :
            a2, b2 = 1042 / 550, -1296
            c2, d2 = 872 / 450, -687
            for i in range(len(M)) :
                x, y = M[i][0], M[i][1]
                if x < 1499 and x > 947 and y < 806 and y > 354 :
                    x, y = int(a2 * x + b2), int(c2 * y + d2)
                    L.extend([[x, y]])

        if self.map1 == 'KistaS' :
            a3, b3 = 1042 / 341, -2179
            c3, d3 = 872 / 283, -1038
            for i in range(len(M)) :
                x, y = M[i][0], M[i][1]
                if x < 1218 and x > 876 and y < 719 and y > 336 :
                    x, y = int(a3 * x+ b3), int(c3 *y + d3)
                    L.extend([[x, y]])

        if self.map1 == 'KistaN' :
            a4, b4 = 1042 / 341, -1862
            c4, d4 = 872 / 283, -502
            for i in range(len(M)):
                x, y = M[i][0], M[i][1]
                if x < 1215 and x > 772 and y < 447 and y > 162 :
                    x, y = int(a4 * x + b4), int(c4 * y + d4)
                    L.extend([[x, y]])
                    
        return L
    
    def waypointsReset(self):
        global M
        M = []
        print(M)




    def infinite_loop(self):
        'DATA'
        'Caracteristics of the map #faire un input'
        #'KTHMap triangelparken'
        #LongitMin_KTH = 18.06783
        #LongitMax_KTH = 18.07120
        #LatitMin_KTH = 59.34886
        #LatitMax_KTH = 59.35029
        #fenetreWidth = 1071
        #fenetreHeight = 872
        #EchelleLongit_KTH = ((LongitMax_KTH - LongitMin_KTH) / fenetreWidth)
        #EchelleLatit_KTH = ((LatitMax_KTH - LatitMin_KTH) / fenetreHeight)

        'MaskinParken'
        LongitMin_KTH = 18.06407
        LongitMax_KTH = 18.06744
        LatitMin_KTH = 59.35174
        LatitMax_KTH = 59.35317
        fenetreWidth = 1042
        fenetreHeight = 872
        EchelleLongit_KTH = ((LongitMax_KTH - LongitMin_KTH) / fenetreWidth)
        EchelleLatit_KTH = ((LatitMax_KTH - LatitMin_KTH) / fenetreHeight)

        '''KistaMap'''
        LongitMin_Kista = 17.95024
        LongitMax_Kista = 17.95360
        LatitMin_Kista = 59.40371
        LatitMax_Kista = 59.40515
        fenetreWidth = 1042
        fenetreHeight = 872
        EchelleLongit_Kista = ((LongitMax_Kista - LongitMin_Kista) / fenetreWidth)
        EchelleLatit_Kista = ((LatitMax_Kista - LatitMin_Kista) / fenetreHeight)

        '''187,7 meters = 1042 pixels''' 
        EchelleMPix = 187.7 / 1042


        '''Tool to update the status
        To be removed at the end'''
        variablePosition = 0
        #print('Subscrib orient :', ori)

       
        ''' Information to guide the user '''
        ChooseMap = pygame.image.load('ChooseMap.png').convert()
        self.screen.blit(ChooseMap, (550,780))
        pygame.display.update()
      
        while self.loop :
            self.create_fond()
            global M, V
            print(M)
            
            '''Load the moving point picture'''
            perso = pygame.image.load('system_car.png').convert()
            perso = pygame.transform.scale(perso, (32,12))
            perso.set_colorkey((255,255,255))
            self.position_perso = perso.get_rect()


            '''Point's rotation'''
            if type(ori) == float:
                #Condition fulfilled as soon as the subscriber gets data
                print('Orientation [%f]' %(ori))
                #perso = pygame.transform.rotate(perso, ori )
                perso =rot_center(perso, ori)
                #perso.set_colorkey((255,255,255))


            '''Buttons'''
            self.reset_button.display_button(self.fond)
            self.quit_button.display_button(self.fond)
            self.waypointsStart_button.display_button(self.fond)
            self.waypointsReset_button.display_button(self.fond)
            self.Kista_button.display_button(self.fond)
            self.KTH_button.display_button(self.fond)
            self.start_button.display_button(self.fond)
            self.stop_button.display_button(self.fond)


            for event in pygame.event.get():
                ''' Scan the pygame events such as mouse clicks or keyboard's buttons pressed  '''
                if event.type == pygame.MOUSEBUTTONDOWN:
                    ''' Activation of the buttons at mouse button down '''
                    self.reset_button.update_button(self.fond, action=self.reset)
                    self.quit_button.update_button(self.fond, action=gamequit)
                    self.waypointsStart_button.update_button(self.fond, action=self.waypoints_personalized)
                    self.waypointsReset_button.update_button(self.fond, action=self.waypointsReset)
                    self.Kista_button.update_button(self.fond, action=self.Display_KistaMap)
                    self.KTH_button.update_button(self.fond, action=self.Display_KTHMap)
                    self.start_button.update_button(self.fond, action=start)
                    self.stop_button.update_button(self.fond, action=stop)

                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 5 and event.pos[0] > 500 :
                    ''' Loop to zoom out with the map. The map's minimal abscissa is 500, the right mouse button corresponds to event.button = 3 '''
                    self.zoom = 1
                    print('bouton droit')
                    if self.map1 == 'KTHE' or self.map1 == 'KTHW' :
                        ''' Displays KTH's map, the scale and the North '''
                        KTHMap = pygame.image.load('Maskinparken.png').convert()
                        pygame.draw.polygon(KTHMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                        pygame.draw.polygon(KTHMap,(0,0,0), ((995,855),(1000,855),(1000,820), (1010,855),(1020,855),(1020, 815), (1015,815), (1015,850), (1005,815), (995,815)))
                        pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000, 840], 2)
                        pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
                        pygame.draw.line(KTHMap, BLACK,[1000, 840], [1000, 837], 2)
                        self.screen.blit(KTHMap, (500,0))
                        self.map1 = 'KTHMap'

                        if Click == True:
                            '''  Displays the moving image '''
                            self.screen.blit(perso, self.position_perso)
                            #self.screen.blit(fleche, self.position_arrow)

                    elif self.map1 == 'KistaN' or self.map1 == 'KistaS' :
                        ''' Displays Kista's map, the scale and the North '''
                        KistaMap = pygame.transform.scale(pygame.image.load('mapKista.png').convert(), (1042,872))
                        pygame.draw.polygon(KistaMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                        pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000, 840], 2)
                        pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
                        pygame.draw.line(KistaMap, BLACK,[1000, 840], [1000, 837], 2)
                        self.screen.blit(KistaMap, (500,0))
                        self.map1 = 'KistaMap'

                        if Click == True:
                            '''  Displays the moving image '''
                            self.screen.blit(perso, self.position_perso)
                            #self.screen.blit(fleche, self.position_arrow)

                if event.type == MOUSEBUTTONDOWN and event.button == 4 and event.pos[0] > 500 and event.pos[1] < 436 :
                    ''' Loop to zoom in with the map. The map's minimal abscissa is 500, the left mouse button corresponds to event.button = 1
                    ZOOM NORTH : click on the top middle part of the map, the ordinate of the mouse has to be between 0 and 436 '''
                    self.zoom = 2
                    if self.map1 == 'KistaMap' or self.map1 == 'KistaS' :
                        ''' Displays the North middle part of Kista's map '''
                        KistaN = pygame.transform.scale(pygame.image.load('KistaN.png').convert(),(1042,872))
                        pygame.draw.polygon(KistaN,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                        pygame.draw.line(KistaN, BLACK, [1000-68, 840], [1000, 840], 2)
                        pygame.draw.line(KistaN, BLACK, [1000-68, 840], [1000-68, 837], 2)
                        pygame.draw.line(KistaN, BLACK, [1000, 840], [1000, 837], 2)
                        self.screen.blit(KistaN, (500,0))
                        self.map1 = 'KistaN'

                        if Click == True:
                            '''  Displays the moving image '''
                            self.screen.blit(perso, self.position_perso)

                if event.type == MOUSEBUTTONDOWN and event.button == 4 and event.pos[0] > 500 and event.pos[1] < 436 and self.map1 == 'KTHMap' and self.map1 != 'KTHE' :
                        self.zoom = 2
                        self.map1 = 'KTHW'
                        KTHW = pygame.image.load('Maskinparken-W.png').convert()
                        KTHW = pygame.transform.scale(KTHW, (1042,872))
                        pygame.draw.polygon(KTHW,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                        pygame.draw.line(KTHW, BLACK, [1000-68, 840], [1000, 840], 2)
                        pygame.draw.line(KTHW, BLACK, [1000-68, 840], [1000-68, 837], 2)
                        pygame.draw.line(KTHW, BLACK, [1000, 840], [1000, 837], 2)
                        self.screen.blit(pygame.transform.scale(pygame.image.load('Maskinparken-W.png').convert(), (1042,872)), (500,0))


                        if Click == True:
                            '''  Displays the moving image '''
                            self.screen.blit(perso, self.position_perso)

                if event.type == MOUSEBUTTONDOWN and event.button == 4 and event.pos[0] > 500 and event.pos[1] > 436 and self.map1 == 'KTHMap'and self.map1 != 'KTHW' :
                    self.zoom = 2
                    KTHE = pygame.image.load('Maskinparken-E.png').convert()
                    KTHE = pygame.transform.scale(KTHE, (1042,872))
                    self.screen.blit(KTHE, (500,0))
                    self.map1 = 'KTHE'

                    if Click == True :
                        '''  Displays the moving image '''
                        #print('Position perso', self.position_perso.centerx,self.position_perso.centery)
                        self.screen.blit(perso, self.position_perso)

     
                        #ZOOM SUD
                if event.type == MOUSEBUTTONDOWN and event.button == 4 and event.pos[0] > 500 and event.pos[1] > 436 :
                    self.zoom = 2
                    if self.map1 == 'KistaMap' or self.map1 == 'KistaN' :
                        self.map1 = 'KistaS'
                        KistaS = pygame.image.load('KistaS.png').convert()
                        KistaS = pygame.transform.scale(KistaS,(1042,872))
                        pygame.draw.polygon(KistaS,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                        pygame.draw.line(KistaS, BLACK, [1000-68, 840], [1000, 840], 2)
                        pygame.draw.line(KistaS, BLACK, [1000-68, 840], [1000-68, 837], 2)
                        pygame.draw.line(KistaS, BLACK, [1000, 840], [1000, 837], 2)
                        self.screen.blit(KistaS, (500,0))
                        print('My map is here', self.map1)

                        if Click == True :
                            '''  Displays the moving image '''
                            self.screen.blit(perso, self.position_perso)


                if event.type == MOUSEBUTTONDOWN and event.button == 1 and event.pos[0] > 500 :
                    ''' Detect a mouse click on the map and store the position in pixels in the M list '''
                    global M
                    if self.zoom == 1:
                        print('zoom1')
                        x_mouse, y_mouse = pygame.mouse.get_pos()[0], pygame.mouse.get_pos()[1]
                        M.extend([[x_mouse, y_mouse]])
                    if self.zoom == 2:
                        print('zoom2')
                        a1, b1 = 1042 / 486, -604
                        c1, d1 = 872 / 404, -28
                        a2, b2 = 1042 / 341, -2179
                        c2, d2 = 872 / 283, -1038
                        a4, b4 = 1042 / 550, -1296
                        c4, d4 = 872 / 450, -687
                        a3, b3 = 1042 / 341, -1862
                        c3, d3 = 872 / 283, -502
                        s, t = pygame.mouse.get_pos()[0], pygame.mouse.get_pos()[1]
                        if self.map1 == 'KTHW' :
                            x_mouse, y_mouse = int((s - b1) / a1), int((t - d1) / c1)
                            M.extend([[x_mouse, y_mouse]])
                        if self.map1 == 'KTHE' :
                            x_mouse, y_mouse = int((s - b4) / a4), int((t - d4) / c4)
                            M.extend([[x_mouse, y_mouse]])
                        if self.map1 == 'KistaS' :
                            x_mouse, y_mouse = int((s - b2) / a2), int((t - d2) / c2)
                            M.extend([[x_mouse, y_mouse]])
                        if self.map1 == 'KistaN' :
                            x_mouse, y_mouse = int((s - b3) / a3), int((t - d3) / c3)
                            M.extend([[x_mouse, y_mouse]])

                if event.type == MOUSEBUTTONDOWN and event.button == 3 and event.pos[0] > 500 :
                    ''' To remove waypoints from the M list : detect a click near a waypoint and remove it from the M list '''

                    x, y = pygame.mouse.get_pos()[0], pygame.mouse.get_pos()[1]
                    if self.zoom == 1:
                        Remove = False
                        for i in range(len(M)) :
                                                        
                            r2 = ((x - M[i][0]) ** 2) + ((y - M[i][1]) ** 2)
                            if r2 <= 25 :
                                Remove,a = True, i
                        if Remove :
                            M.remove([M[a][0], M[a][1]])

                    if self.zoom == 2:
                        Remove = False
                        L = self.WaypointsZoom()
                        print(L)
                        for i in range(len(M)) :
                            print(L[i][0], i, len(L), len(L[i]), L)
                            r2 = ((x - L[i][0]) ** 2) + ((y - L[i][1]) ** 2)
                            if r2 <= 25 :
                                Remove, a = True, i
                        if Remove :
                            M.remove([M[a][0], M[a][1]])

            ''' Points sent by the car - Conversion GPS > pixels'''
            if L_car != None and l_car != None :
                x_KTH, y_KTH = int((((L_car - LongitMin_KTH) / EchelleLongit_KTH)) + 500), int((LatitMax_KTH - l_car) / EchelleLatit_KTH)
                print(x_KTH, y_KTH)
                x_Kista, y_Kista = int(((L_car - LongitMin_Kista) / EchelleLongit_Kista) +500 ), int((LatitMax_Kista - l_car) / EchelleLatit_Kista)

                a1, b1 = 1042 / 486, -604
                c1, d1 = 872 / 404, -28
                if x_KTH < 1002 and x_KTH > 514 and y_KTH < 418 and y_KTH > 12 :
                    x_KTHW, y_KTHW = int(a1 * x_KTH + b1), int(c1 * y_KTH + d1)

                a2, b2 = 1042 / 341, -2179
                c2, d2 = 872 / 283, -1038
                if x_Kista < 1218 and x_Kista > 978 and y_Kista > 336 and y_Kista < 719 :
                    x_KistaS, y_KistaS = int(a2 * x_Kista + b2), int(c2 * y_Kista + d2)

                a4, b4 = 1042 / 550, -1296
                c4, d4 = 872 / 450, -687
                if x_KTH < 1499 and x_KTH > 947 and y_KTH < 806 and y_KTH > 354 :
                    x_KTHE, y_KTHE = int(a4 * x_KTH + b4), int(c4 * y_KTH + d4)

                a3, b3 = 1042 / 341, -1862
                c3, d3 = 872 / 283, -502
                if x_Kista > 772 and x_Kista < 1215 and y_Kista > 162 and y_Kista < 447 :
                    x_KistaN, y_KistaN = int(a3 * x_Kista + b3), int(c3 * y_Kista + d3)

            ''' Position Affectation '''
            if self.zoom == 1 and l_car != None and L_car != None and self.map1 == 'KistaMap' :
                    #pygame.draw.circle( self.screen, GREEN, (xS_Kista, yS_Kista), 5, 0)
                    #print(x_Kista, y_Kista)
                    self.position_perso.centerx, self.position_perso.centery = x_Kista, y_Kista

            if self.zoom == 1 and l_car != None and L_car != None and self.map1 == 'KTHMap' :
                    #pygame.draw.circle(self.screen, GREEN, (xS_KTH, yS_KTH), 5, 0)
                    self.position_perso.centerx, self.position_perso.centery = x_KTH, y_KTH

            
            if self.zoom == 2 and l_car != None and L_car != None and self.map1 == 'KTHW' :
                '''KTHN
                bottomright=(1045,1349) resized at (1073,739)
                Longueur, largeur = 549,521
                points donnes en echelle 1 '''
                if x_KTH > 514 and x_KTH < 1002 and y_KTH > 12 and y_KTH < 418 :
                    self.position_perso.centerx, self.position_perso.centery = x_KTHW, y_KTHW

            if self.zoom == 2 and l_car != None and L_car != None and self.map1 == 'KTHE' :
                '''KTHE
                topleft=(1345,819)
                Longueur, largeur = 433,561 (voir Paint)
                points donnes en echelle 1 '''
                if x_KTH > 947 and x_KTH < 1499 and y_KTH > 354 and y_KTH < 806 :
                    self.position_perso.centery = y_KTHE
                    self.position_perso.centerx = x_KTHE

            if self.zoom == 2 and l_car != None and L_car != None and self.map1 == 'KistaN' :
                '''KistaN
                topleft=(284,156)
                Longueur, largeur = 399,417 (voir Paint)
                points donnes en echelle 1 '''
                if x_Kista > 872 and x_Kista < 1215 and y_Kista > 162 and y_Kista < 447 :
                    self.position_perso.centerx = x_KistaN
                    self.position_perso.centery = y_KistaN

            if self.zoom == 2 and l_car != None and L_car != None and self.map1 == 'KistaS' :
                if x_Kista < 1218 and x_Kista > 876 and y_Kista > 336 and y_Kista < 719 :
                    self.position_perso.centery = y_KistaS
                    self.position_perso.centerx = x_KistaS


            '''Refresh the display by displaying the maps and the point to avoid having several images of the point on the map'''

            '''Kista - North'''
            if self.map1 == 'KistaN' :
                KistaN = pygame.transform.scale(pygame.image.load('KistaN.png').convert(), (1042, 872))
                pygame.draw.polygon(KistaN,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                pygame.draw.polygon(KistaN,(0,0,0), ((1000,795),(1005,795),(1005,780), (1015,795),(1023,795),(1023, 775), (1018,775), (1018,790), (1008,775), (1000,775)))
                #pygame.draw.polygon(KistaN, (0,0,0), (x,y), (x+16,y), (x+16, y+4), (10+x, y+4), (x+10, y+22), (6+x, y+22), (x, y+15), (x, y+13), (x+4, 13+y))
                pygame.draw.line(KistaN, BLACK, [1000-68, 840], [1000, 840], 2)
                pygame.draw.line(KistaN, BLACK, [1000-68, 840], [1000-68, 837], 2)
                pygame.draw.line(KistaN, BLACK, [1000, 840], [1000, 837], 2)
                self.screen.blit(KistaN, (500,0))

                ''' Display the waypoints '''
                L = self.WaypointsZoom()
                for i in range(len(L)):
                    x, y = L[i][0], L[i][1]
                    pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)
                
                if Click == True :
                    ''' Display the moving point '''
                    perso = pygame.transform.scale(perso, (43,12))
                    self.screen.blit(perso, self.position_perso)  #Default value in constructor if no GPS reception

            '''KTH - West'''
            if self.map1 == 'KTHW' :
                KTHW = pygame.transform.scale(pygame.image.load('Maskinparken-W.png').convert(), (1042,872))
                pygame.draw.polygon(KTHW,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                pygame.draw.polygon(KTHW,(0,0,0), ((1000,795),(1005,795),(1005,780), (1015,795),(1023,795),(1023, 775), (1018,775), (1018,790), (1008,775), (1000,775)))
                pygame.draw.line(KTHW, BLACK, [1000-68, 840], [1000, 840], 2)
                pygame.draw.line(KTHW, BLACK, [1000-68, 840], [1000-68, 837], 2)
                pygame.draw.line(KTHW, BLACK, [1000, 840], [1000, 837], 2)
                self.screen.blit(KTHW, (500,0))

                ''' Display the waypoints '''
                L = self.WaypointsZoom()
                for i in range(len(L)):
                    x, y = L[i][0], L[i][1]
                    pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)

                if Click == True :
                    ''' Display the moving point '''
                    perso = pygame.transform.scale(perso, (57, 18))
                    self.screen.blit(perso, self.position_perso)

            '''Kista - South'''
            if self.map1 == 'KistaS' :
                KistaS = pygame.image.load('KistaS.png').convert()
                KistaS = pygame.transform.scale(KistaS,(1042,872))
                pygame.draw.polygon(KistaS,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                pygame.draw.polygon(KistaS,(0,0,0), ((1000,795),(1005,795),(1005,780), (1015,795),(1023,795),(1023, 775), (1018,775), (1018,790), (1008,775), (1000,775)))
                pygame.draw.line(KistaS, BLACK, [1000 - 68, 840], [1000, 840], 2)
                pygame.draw.line(KistaS, BLACK, [1000 - 68, 840], [1000 - 68, 837], 2)
                pygame.draw.line(KistaS, BLACK, [1000, 840], [1000, 837], 2)
                self.screen.blit(KistaS, (500,0))


                ''' Display the waypoints  '''
                L = self.WaypointsZoom()
                for i in range(len(L)):
                    x, y = L[i][0], L[i][1]
                    pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)

                if Click == True:
                    '''  Displays the moving image '''
                    perso = pygame.transform.scale(perso, (42,12))
                    self.screen.blit(perso, self.position_perso)

            ''' KTH - East '''        
            if self.map1 == 'KTHE' :
                KTHE=pygame.transform.scale(pygame.image.load('Maskinparken-E.png').convert(), (1042,872))
                pygame.draw.polygon(KTHE,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                pygame.draw.polygon(KTHE,(0,0,0), ((1000,795),(1005,795),(1005,780), (1015,795),(1023,795),(1023, 775), (1018,775), (1018,790), (1008,775), (1000,775)))
                pygame.draw.line(KTHE, BLACK, [1000 - 68, 840], [1000, 840], 2)
                pygame.draw.line(KTHE, BLACK, [1000 - 68, 840], [1000 - 68, 837], 2)
                pygame.draw.line(KTHE, BLACK, [1000, 840], [1000, 837], 2)
                self.screen.blit(KTHE, (500,0))

                ''' Display the waypoints of the scenario 1 or the customized scenario '''
                L = self.WaypointsZoom()
                for i in range(len(L)):
                    x, y = L[i][0], L[i][1]
                    pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)

                if Click == True :
                    '''  Displays the moving image '''
                    perso = pygame.transform.scale(perso, (72, 17))
                    self.screen.blit(perso, self.position_perso)

            ''' KTH '''
            if self.map1 == 'KTHMap' :
                KTHMap = pygame.image.load('Maskinparken.png').convert()
                pygame.draw.polygon(KTHMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                pygame.draw.polygon(KTHMap,(0,0,0), ((1000,795),(1005,795),(1005,780), (1015,795),(1023,795),(1023, 775), (1018,775), (1018,790), (1008,775), (1000,775)))
                pygame.draw.line(KTHMap, BLACK, [1000 - 68, 840], [1000, 840], 2)
                pygame.draw.line(KTHMap, BLACK, [1000 - 68, 840], [1000 - 68, 837], 2)
                pygame.draw.line(KTHMap, BLACK, [1000, 840], [1000, 837], 2)
                self.screen.blit(KTHMap, (500,0))

                if Click == True :
                    '''  Displays the moving image '''
                    perso = pygame.transform.scale(perso, (32,12))
                    #print(self.position_perso)
                    self.screen.blit(perso, self.position_perso)


                ''' Display waypoints of custumized scenario '''

                for i in range(len(M)):
                    ''' Display the waypoints '''
                    x, y = M[i][0], M[i][1]
                    pygame.draw.circle(self.screen, ORANGE, (x, y), 5, 0)

            ''' Kista '''
            if self.map1 == 'KistaMap' :
                KistaMap = pygame.image.load('mapKista.png').convert()
                pygame.draw.polygon(KistaMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                pygame.draw.polygon(KistaMap,(0,0,0), ((1000,795),(1005,795),(1005,780), (1015,795),(1023,795),(1023, 775), (1018,775), (1018,790), (1008,775), (1000,775)))
                pygame.draw.line(KistaMap, BLACK, [1000 - 68, 840], [1000, 840], 2)
                pygame.draw.line(KistaMap, BLACK, [1000 - 68, 840], [1000 - 68, 837], 2)
                pygame.draw.line(KistaMap, BLACK, [1000, 840], [1000, 837], 2)
                self.screen.blit(KistaMap, (500,0))

                if Click == True:
                    '''  Displays the moving image '''
                    perso = pygame.transform.scale(perso, (16,6))
                    #print(self.position_perso.centerx, self.position_perso.centery, self.position_perso)
                    self.screen.blit(perso, self.position_perso)

                
                ''' Display waypoints of custumized scenario '''

                for i in range(len(M)):
                    ''' Display the waypoints '''
                    x, y = M[i][0], M[i][1]
                    pygame.draw.circle(self.screen, ORANGE, (x, y), 5, 0)

            if event.type == MOUSEBUTTONDOWN:
                print('MOUSEBUTTONDOWN', event.pos[1])


           

            ''' Display the quality of the GPS data '''
            if GPSQuali == 'fix' :
                green_square = pygame.transform.scale(pygame.image.load('green_square.png').convert(), (100, 50))
                self.screen.blit(self.fond, (315, 245))

            if GPSQuali == 'float':
                orange_square = pygame.transform.scale(pygame.image.load('orange_square.png').convert(), (100, 50))
                self.screen.blit(self.fond, (315, 245))
            
            if GPSQuali == 'single' :
                red_square = pygame.transform.scale(pygame.image.load('red_square.png').convert(), (100, 50))
                self.screen.blit(self.fond, (315, 245))
            if GPSQuali != 'fix' and GPSQuali != 'float' and GPSQuali != 'single' :
                self.VarMess = 'No information about GPS quality'

            ''' Obstacle detection '''
            if Obstacle == 'True':
                self.obt_detec = 'True'
            elif Obstacle == 'False':
                self.obt_detec = 'False'

            ''' Velocity display '''
            if V != None :
                V = float(int(V * 10 ))/ 10
                self.level = V
            else :
                self.VarMess = ' No velocity input '



            self.update_textes()
            for text in self.textes:

                self.display_text(text[0], text[1], text[2],
                                        text[3], text[4])

            # Ajout du fond dans la fenêtre
            self.screen.blit(self.fond, (0, 0))

            t = clock.get_time() / 1000
           
            ''' Update the status '''
            ''' Detect if the data received is updated between 2 loops '''
            if variablePosition % 2 == 0 and l_car != None and L_car != None :
                StockPosition[0][0], StockPosition[0][1] = self.position_perso.centerx, self.position_perso.centery
                variablePosition += 1
            elif variablePosition % 2 == 1 and l_car != None and L_car != None :
                StockPosition[1][0], StockPosition[1][1] = self.position_perso.centerx, self.position_perso.centery
                variablePosition += 1
          
            if l_car!=None and L_car!=None :
                self.status = 'GPS reception'
            if  StockPosition[0][0]-StockPosition[1][0]==0 and StockPosition[0][1]-StockPosition[1][1]==0:
                self.status = 'No GPS reception'
               

            # Actualisation de l'affichage
            pygame.display.update()
            # 10 fps
            clock.tick(10)




def start():
    print("start")
    boo = Bool()
    boo.data = True
    EmergencyStopPublisher.publish(boo)

def stop():
    print("stop")
    boo = Bool()
    boo.data = False
    EmergencyStopPublisher.publish(boo)




def gamequit():
    print("Quit")
    pygame.quit()
    sys.exit()


if __name__ == '__main__':
    game = Game()
    game.infinite_loop()

   
