#! /usr/bin/env python3
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



#talk about variables

from __future__ import division
import pygame
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3, PoseArray, Pose
from turtlesim.msg import Pose as Pos
import datetime
from nav_msgs.msg import Odometry
from pygame.locals import *
from math import sqrt
import numpy as np
#from qualisys.msg import Subject


pygame.init()
clock = pygame.time.Clock()

BLACK = 0, 0, 0
WHITE = 255, 255, 255
CIEL = 0, 200, 255
RED = 185, 18, 27
ORANGE = 255, 100, 0
GREEN = 0, 255, 0
GREEN2 = 32, 178, 15
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
state_x = None
state_y = None
ori = None
Tableau2 = [] #Pour les positions relatives du scenario personalise
Tableau3 = [] #Pour les positions en pixels du scenario personalise
Click = True
GPSQuali = 'Rien'
L = []
Obstacle = 'False'

'''For velocity calculation, intialization '''
StockPosition = [[0,0],[0,0]]


'''Convert orientation received from nodes'''
def constrainAngle(x):
	return x % 2 * np.pi

def convertRad2Deg(x):
	return(180 * x) / np.pi

'''Define the subscriber to the velocity'''
def callbackVel(msg):
    global V
    #rospy.loginfo("Received a /cmd_vel message!")
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s", Twist.linear)
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    #rospy.loginfo("Conversion : x lineaire : [%f], x converti: [%f]" %(msg.linear.x, msg.linear.x*10))
    V = msg.linear.x
    
Subs = rospy.Subscriber("/turtle1/cmd_vel", Twist, callbackVel)

'''Define the subscriber to the position'''
def callbackPose(msg):
    global X, Y
    #rospy.loginfo("Received a /Pose message !!")
    #rospy.loginfo("Position de la tortue : [%f, %f]" %(msg.x, msg.y) )
    X = msg.x
    Y = msg.y

SubsPose = rospy.Subscriber("/turtle1/pose", Pos, callbackPose)

'''Define the subscriber to the position'''
def callbackCar_State(msg):
    global state_x, state_y, ori
    #rospy.loginfo("Received a simulation message !!")
    #rospy.loginfo("Position en rospy : [%f, %f]" %(msg.pose.pose.position.x, msg.pose.pose.position.y) )
    state_x = msg.pose.pose.position.x
    state_y = msg.pose.pose.position.y
    ori = msg.pose.pose.orientation.w
    ori = constrainAngle(ori)
    ori = convertRad2Deg(ori)
    #rospy.loginfo("recu orientation: [%f]" %(ori))
    #rospy.loginfo("recu position x: [%f, %f]" %(state_x, state_y))



SubsCar_State = rospy.Subscriber("/car_state_topic", Odometry, callbackCar_State)
#SubsCar_State=rospy.Subscriber("qualisys/F1Tenth", Subject, callbackCar_State)
#SubsCar_State=rospy.Subscriber("odometry/filtered", Odometry, callbackCar_State)

'''Define the subscriber to the GPS Status'''
'''def callbackGPSQuality(msg):
    global GPSQuali
    GPSQuali = msg.solution
    rospy.loginfo("recu GPS string messages [%s]" %(GPSQuali))

SubsGPSQuality = rospy.Subscriber("/gps_reach", gps_reach, callbackGPSQuality)'''

'''Define the subscriber to the Obstacle detection'''
'''def callbackObst_Detect(msg):
    global Obstacle

    Obstacle = msg.flag

SubsObst_Detect = rospy.Subscriber("/detect_result", Cmd, callbackCar_State)'''

'''Initialize publisher of the waypoints'''
goalPublisher = rospy.Publisher('waypoints', PoseArray, queue_size = 10)

'''Date''' 
date = datetime.datetime.now()


'''Waypoints coordinates for scenario 1'''
Tableau1 = [[995, 789], [995, 670], [995, 550],[995, 802], [995, 728], [995, 200],[995, 127],[995, 186],[995, 217],[995, 42]]


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
        self.screen = pygame.display.set_mode((1571, 872))
        self.level = 0.0 
        self.loop = True
        self.position_perso = (600,600)
        self.position_arrow = (615,615)
        self.status = 'Waiting to begin'
        self.scale = 1

        ''' Font definition'''
        self.big = pygame.font.SysFont('freesans', 48)
        self.small = pygame.font.SysFont('freesans', 36)
        self.mediumsmall = pygame.font.SysFont('freesans',28)
        self.verysmall = pygame.font.SysFont('freesans', 10)
        self.map1 = 'KistaMap' '''defaultValue'''

        self.create_fond()
        self.create_button()
        self.zoom = 1  #1 if zoom out, 2 if zoom in
        ''' Detecteurs de scenarios '''
        self.Scene1 = False
        self.CustScene = False

        self.obt_detec = 'False'

    def update_textes(self):
        self.textes = [ ["Autonomous Driving with F1/10", ORANGE, self.small, 0, 50],
                        ["Speed (m/s) :", BLACK, self.small, -65, 150],
                        [str(self.level), BLACK, self.small, 90, 150],
                        ["Status :", BLACK, self.small, -110, 250],
                        [str(self.status), BLACK, self.small, 70, 250],
                        ["GPS quality :", BLACK, self.small, -70, 350],
                        ["Maps :", BLACK, self.small, -110, 450],
                        ["KTH Campus, Stockholm", BLACK, self.verysmall, 160, 840],
                        ["OpenStreetMap ", BLACK, self.verysmall, 142, 850],
                        [str(date), BLACK, self.verysmall, 172, 861],
                        ["Obstacle detection :", BLACK, self.small, -23, 670],
                        [str(self.obt_detec), BLACK, self.small, 180, 670],
                        ]

    def create_fond(self):
        ''' Background '''
        self.fond = pygame.Surface((500, 872))
        ''' Color '''
        self.fond.fill(WHITE)

    def create_button(self):
        self.reset_button = Button(self.fond, "   Reset   ", YELLOW, self.small, -40, 853)
        self.quit_button  = Button(self.fond, "   Quit   ", RED, self.small, -180, 853)
        self.Scenario1_button = Button(self.fond, "  Scenario 1  ", MARRON2, self.small, -80, 550)
        self.Scenario2_button=Button(self.fond,"  Scenario 2  ", MARRON2, self.small, 130,550)
        self.Kista_button = Button(self.fond, "  Kista  ", BEIGE, self.small, 130, 450)
        self.KTH_button = Button(self.fond, "  KTH  ", BEIGE, self.small, 10, 450)
        self.PersoScenario_button = Button(self.fond, "  Customized Scenario  ", MARRON2, self.small, 20, 595)

    def display_text(self, text, color, font, dx, dy):
        '''Add text on the background. dx gap, dy gap in relation to the center.
        '''
        mytext = font.render(text, True, color)  # True for antialiasing
        textpos = mytext.get_rect()
        textpos.centerx = self.fond.get_rect().centerx + dx
        textpos.centery = dy
        self.fond.blit(mytext, textpos)

    def reset(self):
        ''' Display the map (KTH or Kista) to erase drawings or pictures. Maps are displayed with a zoom out (self.zoom=1)'''
        global Tableau2, Tableau3


    	KistaMap = pygame.image.load('mapKista.png').convert()
        KTHMap = pygame.transform.scale(pygame.image.load('mapKTH.png').convert(), (1072,872))
        if self.map1 == 'KTHS' or self.map1 == 'KTHN' or self.map1 == 'KTHMap' :
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

        ''' Detecteurs remis a zero ''' 
        self.Scene1 = False
        self.CustScene = False

        ''' Tableaux de stockage vides '''
        Tableau2 = []
        Tableau3 = []

        #print('Position perso', self.position_perso.centerx,self.position_perso.centery)
    	print("reset")

    def Display_KistaMap(self):
        '''Action of "Kista" button. Display Kista's map with zoom out'''
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
        '''Action of "KTH" button. Display KTH's map with zoom out'''
    	self.zoom = 1
    	global Click
    	Click = True
        KTHMap = pygame.image.load('mapKTH.png').convert()
        KTHMap = pygame.transform.scale(pygame.image.load('mapKTH.png').convert(), (1072,872))
        pygame.draw.polygon(KTHMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
        pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000, 840], 2)
        pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
        pygame.draw.line(KTHMap, BLACK, [1000, 840], [1000, 837], 2)
        self.screen.blit(KTHMap, (500,0))
        self.map1 = 'KTHMap'
        pygame.display.update()


    def Scenario1(self):
        #Displays the waypoints of scenario 1

        #First, let's reset the map in case waypoints were arleady there
        self.reset()
        self.Scene1 = True
        #for i in range (len(Tableau1)):
        #    x,y=Tableau1[i][0], Tableau1[i][1]
        #    pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)
        #    print(i)
        #print(self.Scene1)
        waypointsPublish()


    def CreatePersonalizedScenario(self):
        '''Create waypoints by mouse clicking on the map;
        Press Return to stop the selection and publish the data
        The click öust be on the map : x>500
        x,y : position relative to the top left corner of the display'''

        '''First, let's reset the map in case waypoints were arleady there'''
        self.reset()
        #print(Tableau2, Tableau3)

        '''Information to guide the user'''
        info_custoScenario = pygame.image.load('HMI-Info-CustoScenario.png').convert()
        self.screen.blit(info_custoScenario, (550,780))
        pygame.display.update()

        '''10metres = 68pixels''' 
        EchelleMPix = 10 / 68

        self.CustScene = True
       
        continuer = 1
        while continuer:
            for event in pygame.event.get():
                #print('Get a event msg')
                if event.type == MOUSEBUTTONDOWN and event.button == 1 :
                    #Select and store the data
                    #print('Get a mousebutton msg')
                    x, y = pygame.mouse.get_pos()[0], pygame.mouse.get_pos()[1]
                    if x > 500 :
                        #conversion
                        if len(Tableau2)>0:

                            '''  Fill in Tableau3 with positions in pixels. Draw points. Convert positions into 
                            relative position in relation with the 1st point clicked, position in m. Fill in Tableau 2
                            with relative position in m. '''

                            Tableau3.extend([[x, y]])
                            pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)
                            x, y = (x - Tableau2[0][0])*EchelleMPix, (y - Tableau2[0][1])*EchelleMPix
                            P = [x, y]

                        else:
                            Tableau3.extend([[x, y]])
                            pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)
                            P=[x,y]
                    	Tableau2.extend([P])
                    	pygame.display.update()

                if event.type == KEYDOWN :
                    ''' Define the starting point as the 1st point clicked. Remove the starting point from the list of relative positions.
                    Publish the relative positions and quit  '''
                    yS_KTH, xS_KTH = Tableau2[0][1], Tableau2[0][0]
                    yS_Kista, xS_Kista = Tableau2[0][1], Tableau2[0][0]
                    del Tableau2[0] #remove the 1st point which is the starting point from where the relative position is calculated
                    if event.key == K_RETURN:
                        #Publish the data
                        #print(Tableau2)
                        waypoints_personalized()
                        continuer = 0
        
    def WaypointsZoom(self):
        global L
        L = []

        if self.Scene1 == True :
            Tableau = Tableau1
        if self.CustScene == True :
            Tableau = Tableau3

        if self.map1== 'KTHN' :
            a1, b1 = 1071 / 301, -2837
            c1, d1 = 872 / 286, -875
            for i in range(len(Tableau)):
                x, y = Tableau[i][0], Tableau[i][1]
                if x < 1240 and x > 937 and y < 574 and y > 286 :
                    x, y = int(a1 * x + b1), int(c1 * y + d1)
                    L.extend([[x, y]])
        if self.map1 == 'KTHS' :
            a2, b2 = 1071 / 237, -4018
            c2, d2 = 872 / 307, -1275
            for i in range(len(Tableau)) :
                x, y = Tableau[i][0], Tableau[i][1]
                if x < 1238 and x > 999 and y < 757 and y > 448 :
                    x, y = int(a2 * x + b2), int(c2 * y + d2)
                    L.extend([[x, y]])

        if self.map1 == 'KistaS' :
            a3, b3 = 1071 / 414, -1859
            c3, d3 = 872 / 419, -636
            for i in range(len(Tableau)) :
                x, y = Tableau[i][0], Tableau[i][1]
                if x < 1227 and x > 911 and y < 725 and y > 305 :
                    x, y = int(a3 * x+ b3), int(c3 *y + d3)
                    L.extend([[x, y]])

        if self.map1 == 'KistaN' :
            a4, b4 = 1071 / 399, -1604
            c4, d4 = 872 / 417, -326
            for i in range(len(Tableau)):
                x, y = Tableau[i][0], Tableau[i][1]
                if x < 1184 and x > 783 and y < 573 and y > 156 :
                    x, y = int(a4 * x + b4), int(c4 * y + d4)
                    L.extend([[x, y]])
        return L



    def infinite_loop(self):

    	'''DATA
    	Caracteristics of the map #faire un input
    	KTHMap'''
    	LongitMin_KTH = 18.06783
    	LongitMax_KTH = 18.07120
    	LatitMin_KTH = 59.34886
    	LatitMax_KTH = 59.35029
    	fenetreWidth = 1071
    	fenetreHeight = 872
    	EchelleLongit_KTH = ((LongitMax_KTH - LongitMin_KTH) / fenetreWidth)
    	EchelleLatit_KTH = ((LatitMax_KTH - LatitMin_KTH) / fenetreHeight)

    	'''KistaMap'''
    	LongitMin_Kista = 17.95017
    	LongitMax_Kista = 17.95354
    	LatitMin_Kista = 59.40374
    	LatitMax_Kista = 59.40517
    	fenetreWidth = 1071
    	fenetreHeight = 872
    	EchelleLongit_Kista = ((LongitMax_Kista - LongitMin_Kista) / fenetreWidth)
    	EchelleLatit_Kista = ((LatitMax_Kista - LatitMin_Kista) / fenetreHeight)

    	'''10metres = 68pixels''' 
    	EchelleMPix = 10 / 68

        '''Arbitrary Starting Point 
        KTH'''
        lS_KTH, LS_KTH = 59.349378, 18.069872
        #NORDlS_KTH, LS_KTH = 59.349451, 18.069778
        #lS_Kista, LS_Kista = 59.404503, 17.951875
        lS_Kista, LS_Kista = 59.404227, 17.952103

        '''Tool to calculate de velocity
        To be removed at the end'''
        variablePosition = 0
        #print('Subscrib orient :', ori)

        '''CONVERSION LATITUDE LONGITUDE > PIXELS
        xs and yS : arbitrary starting point ''' 
        #yS_KTH, xS_KTH = int((LatitMax_KTH - lS_KTH) / EchelleLatit_KTH), int((((LS_KTH - LongitMin_KTH) / EchelleLongit_KTH)) + 500)
        #yS_Kista, xS_Kista = int((LatitMax_Kista - lS_Kista) / EchelleLatit_Kista), int((((LS_Kista - LongitMin_Kista) / EchelleLongit_Kista)) + 500)

        ''' Information to guide the user '''
        ChooseMap = pygame.image.load('ChooseMap.png').convert()
        self.screen.blit(ChooseMap, (550,780))
        pygame.display.update()
      
        while self.loop :
            self.create_fond()
            
            '''Load the moving point picture'''
            perso = pygame.image.load('system_car.png').convert()
            perso.set_colorkey((255,255,255))
            perso = pygame.transform.scale(perso, (42,15))
            self.position_perso = perso.get_rect()

            #Load the arrow
            #fleche=pygame.image.load('fleche.jpg').convert()
            #fleche=pygame.transform.scale(fleche, (25,25))
            #self.position_arrow=fleche.get_rect()
            #self.position_arrow.centerx, self.position_arrow.centery=self.position_perso.centerx,self.position_perso.centery

            '''Point's rotation'''
            if type(ori) == float:
                #Condition fulfilled as soon as the subscriber gets data
                print('Orientation [%f]' %(ori))
                perso = pygame.transform.rotate(perso, ori )
                perso.set_colorkey((255,255,255))

                      


            '''Buttons'''
            self.reset_button.display_button(self.fond)
            self.quit_button.display_button(self.fond)
            self.Scenario1_button.display_button(self.fond)
            self.Scenario2_button.display_button(self.fond)
            self.Kista_button.display_button(self.fond)
            self.KTH_button.display_button(self.fond)
            self.PersoScenario_button.display_button(self.fond)


            for event in pygame.event.get():
                ''' Scan the pygame events such as mouse clicks or keyboard's buttons pressed  '''
                if event.type == pygame.MOUSEBUTTONDOWN:
                    ''' Activation of the buttons at mouse button down '''
                    self.reset_button.update_button(self.fond, action=self.reset)
                    self.quit_button.update_button(self.fond, action=gamequit)
                    self.Scenario1_button.update_button(self.fond, action=self.Scenario1)
                    self.Scenario2_button.update_button(self.fond, action=waypointsPublish2)
                    self.Kista_button.update_button(self.fond, action=self.Display_KistaMap)
                    self.KTH_button.update_button(self.fond, action=self.Display_KTHMap)
                    self.PersoScenario_button.update_button(self.fond, action=self.CreatePersonalizedScenario)

                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3 and event.pos[0] > 500 :
                    ''' Loop to zoom out with the map. The map's minimal abscissa is 500, the right mouse button corresponds to event.button = 3 '''
                    self.zoom = 1
                    print('bouton droit')
                    if self.map1 == 'KTHS' or self.map1 == 'KTHN' :
                        ''' Displays KTH's map, the scale and the North '''
                        KTHMap = pygame.transform.scale(pygame.image.load('mapKTH.png').convert(), (1072,872))
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
                        KistaMap = pygame.transform.scale(pygame.image.load('mapKista.png').convert(), (1072,872))
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

                if event.type == MOUSEBUTTONDOWN and event.button == 1 and event.pos[0] > 500 and event.pos[1] < 436 :
                    ''' Loop to zoom in with the map. The map's minimal abscissa is 500, the left mouse button corresponds to event.button = 1
                    ZOOM NORTH : click on the top middle part of the map, the ordinate of the mouse has to be between 0 and 436 '''
                    self.zoom = 2
                    if self.map1 == 'KistaMap' or self.map1 == 'KistaS' :
                        ''' Displays the North middle part of Kista's map '''
                        KistaN = pygame.transform.scale(pygame.image.load('KistaN.png').convert(),(1072,872))
                        pygame.draw.circle(self.screen, ORANGE, ( xS_Kista, yS_Kista), 5, 0)
                        pygame.draw.polygon(KistaN,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                        pygame.draw.line(KistaN, BLACK, [1000-68, 840], [1000, 840], 2)
                        pygame.draw.line(KistaN, BLACK, [1000-68, 840], [1000-68, 837], 2)
                        pygame.draw.line(KistaN, BLACK, [1000, 840], [1000, 837], 2)
                        self.screen.blit(KistaN, (500,0))
                        self.map1 = 'KistaN'

                        if Click == True:
                            '''  Displays the moving image '''
                            self.screen.blit(perso, self.position_perso)

                if event.type == MOUSEBUTTONDOWN and event.button == 1 and event.pos[0] > 500 and event.pos[1] < 436 and self.map1 == 'KTHMap' and self.map1 != 'KTHS' :
                        self.zoom = 2
                        self.map1 = 'KTHN'
                        KTHN = pygame.image.load('KTHN.png').convert()
                        KTHN = pygame.transform.scale(KTHN, (1072,872))
                        pygame.draw.circle(self.screen, ORANGE, ( xS_KTH, yS_KTH), 5, 0)
                        pygame.draw.polygon(KTHN,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                        pygame.draw.line(KTHN, BLACK, [1000-68, 840], [1000, 840], 2)
                        pygame.draw.line(KTHN, BLACK, [1000-68, 840], [1000-68, 837], 2)
                        pygame.draw.line(KTHN, BLACK, [1000, 840], [1000, 837], 2)
                        self.screen.blit(pygame.transform.scale(pygame.image.load('KTHN.png').convert(), (1072,872)), (500,0))


                        if Click == True:
                            '''  Displays the moving image '''
                            self.screen.blit(perso, self.position_perso)

                if event.type == MOUSEBUTTONDOWN and event.button == 1 and event.pos[0] > 500 and event.pos[1] > 436 and self.map1 == 'KTHMap'and self.map1 != 'KTHN' :
                    self.zoom = 2
                    KTHS = pygame.image.load('KTHS.png').convert()
                    KTHS = pygame.transform.scale(KTHS, (1072,872))
                    self.screen.blit(KTHS, (500,0))
                    self.map1 = 'KTHS'

                    if Click == True :
                        '''  Displays the moving image '''
                        print('Position perso', self.position_perso.centerx,self.position_perso.centery)
                        self.screen.blit(perso, self.position_perso)

     
            			#ZOOM SUD
            	if event.type == MOUSEBUTTONDOWN and event.button == 1 and event.pos[0] > 500 and event.pos[1] > 436 :
            		self.zoom = 2
            		if self.map1 == 'KistaMap' or self.map1 == 'KistaN' :
            			self.map1 = 'KistaS'
            			KistaS = pygame.image.load('KistaS.png').convert()
            			KistaS = pygame.transform.scale(KistaS,(1072,872))
            			pygame.draw.polygon(KistaS,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
            			pygame.draw.line(KistaS, BLACK, [1000-68, 840], [1000, 840], 2)
            			pygame.draw.line(KistaS, BLACK, [1000-68, 840], [1000-68, 837], 2)
            			pygame.draw.line(KistaS, BLACK, [1000, 840], [1000, 837], 2)
            			self.screen.blit(KistaS, (500,0))
                        print('My map is here', self.map1)

                        if Click == True :
                            '''  Displays the moving image '''
                            self.screen.blit(perso, self.position_perso)


           	
           	'''Determination des positions du point au cours du temps a partir de la simulation de Sud (position relative en m)
           	CONVERSION METRES > PIXELS
        	delta_xp et delta_yp sont les positions relatives donnees par la simulation de Sud en m'''
            if self.CustScene == False :
                yS_KTH, xS_KTH = int((LatitMax_KTH - lS_KTH) / EchelleLatit_KTH), int((((LS_KTH - LongitMin_KTH) / EchelleLongit_KTH)) + 500)
                yS_Kista, xS_Kista = int((LatitMax_Kista-lS_Kista)/EchelleLatit_Kista), int((((LS_Kista-LongitMin_Kista)/EchelleLongit_Kista))+500)
            else :
                yS_Kista, xS_Kista = Tableau3[0][1], Tableau3[0][0]
                yS_KTH, xS_KTH = Tableau3[0][1], Tableau3[0][0]

            ''' Pour le scenario personnalisé, le starting point est le 1er point clicke !  '''

            if xS_KTH != None and yS_KTH != None and state_x != None and state_y != None and xS_Kista != None and yS_Kista != None  :
                delta_xp, delta_yp = (state_x) / EchelleMPix, (state_y ) / EchelleMPix
                x_KTH, y_KTH = int(xS_KTH + delta_xp), int(yS_KTH + delta_yp)
                x_Kista, y_Kista = int(xS_Kista + delta_xp), int(yS_Kista + delta_yp)

                a1, b1 = 1071 / 301, -2837
                c1, d1 = 872 / 286, -875
                if x_KTH > 937 and x_KTH < 1240 and y_KTH > 286 and y_KTH < 574:
                    x_KTHN, y_KTHN = int(a1 * x_KTH + b1), int(c1 * y_KTH + d1)

                a2, b2 = 1071 / 414, -1859
                c2, d2 = 872 / 419, -636
                if x_Kista < 1227 and x_Kista > 911 and y_Kista > 305 and y_Kista < 725 :
                    x_KistaS, y_KistaS = int(a2 * x_Kista + b2), int(c2 * y_Kista + d2)

                a3, b3 = 1071 / 399, -1604
                c3, d3 = 872 / 417, -326
                if x_Kista > 783 and x_Kista < 1184 and y_Kista > 156 and y_Kista < 573 :
                    x_KistaN, y_KistaN = int(a3 * x_Kista + b3), int(c3 * y_Kista + d3)

                a4, b4 = 1071 / 237, -4018
                c4, d4 = 872 / 307, -1275
                if x_KTH > 999 and x_KTH < 1238 and y_KTH > 448 and y_KTH < 757 :
                    x_KTHS, y_KTHS = int(a4 * x_KTH + b4), int(c4 * y_KTH + d4)
		
            if self.zoom == 1 and state_x != None and state_y != None and self.map1 == 'KistaMap' :
                    pygame.draw.circle( self.screen, GREEN, (xS_Kista, yS_Kista), 5, 0)
                    self.position_perso.centerx, self.position_perso.centery = x_Kista, y_Kista

            if self.zoom == 1 and state_x != None and state_y != None and self.map1 == 'KTHMap' :
                    pygame.draw.circle(self.screen, GREEN, (xS_KTH, yS_KTH), 5, 0)
                    self.position_perso.centerx, self.position_perso.centery = x_KTH, y_KTH

            
            if self.zoom == 2 and state_x != None and state_y != None and self.map1 == 'KTHN' :
            	'''KTHN
            	bottomright=(1045,1349) resized at (1073,739)
            	Longueur, largeur = 549,521 (voir Paint)
            	points donnes en echelle 1 '''
            	if x_KTH > 937 and x_KTH < 1240 and y_KTH > 286 and y_KTH < 574 :
                    self.position_perso.centerx, self.position_perso.centery = x_KTHN, y_KTHN

            if self.zoom == 2 and state_x != None and state_y != None and self.map1 == 'KTHS' :
            	'''KistaN
            	topleft=(1345,819)
            	Longueur, largeur = 433,561 (voir Paint)
            	points donnes en echelle 1 '''
            	if x_KTH > 999 and x_KTH < 1238 and y_KTH > 448 and y_KTH < 757 :
                    self.position_perso.centery = y_KTHS
                    self.position_perso.centerx = x_KTHS

            if self.zoom == 2 and state_x != None and state_y != None and self.map1 == 'KistaN' :
            	'''KistaN
            	topleft=(284,156)
            	Longueur, largeur = 399,417 (voir Paint)
            	points donnes en echelle 1 '''
            	if x_Kista > 783 and x_Kista < 1184 and y_Kista > 156 and y_Kista < 573 :
                    self.position_perso.centerx = x_KistaN
                    self.position_perso.centery = y_KistaN

            if self.zoom == 2 and state_x != None and state_y != None and self.map1 == 'KistaS' :
                if x_Kista < 1227 and x_Kista > 911 and y_Kista > 305 and y_Kista < 725 :
                    self.position_perso.centery = y_KistaS
                    self.position_perso.centerx = x_KistaS


            '''Refresh the display by displaying the maps and the point to avoid having several images of the point on the map'''
            '''Kista - North'''
            if self.map1 == 'KistaN' :
                KistaN = pygame.transform.scale(pygame.image.load('KistaN.png').convert(), (1072, 872))
                pygame.draw.circle(self.screen, ORANGE, ( xS_Kista, yS_Kista), 5, 0)
                pygame.draw.polygon(KistaN,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                pygame.draw.polygon(KistaN,(0,0,0), ((1000,795),(1005,795),(1005,780), (1015,795),(1023,795),(1023, 775), (1018,775), (1018,790), (1008,775), (1000,775)))
                pygame.draw.line(KistaN, BLACK, [1000-68, 840], [1000, 840], 2)
                pygame.draw.line(KistaN, BLACK, [1000-68, 840], [1000-68, 837], 2)
                pygame.draw.line(KistaN, BLACK, [1000, 840], [1000, 837], 2)
                self.screen.blit(KistaN, (500,0))

                if self.Scene1 == True or self.CustScene == True :
                    L = self.WaypointsZoom()
                    for i in range(len(L)):
                        x, y = L[i][0], L[i][1]
                        pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0) 

                if Click == True :
                    self.screen.blit(perso, self.position_perso)

            '''KTH - North'''
            if self.map1 == 'KTHN' :
                KTHN = pygame.transform.scale(pygame.image.load('KTHN.png').convert(), (1072,872))
                pygame.draw.circle(self.screen, ORANGE, ( xS_KTH, yS_KTH), 5, 0)
                pygame.draw.polygon(KTHN,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                pygame.draw.polygon(KTHN,(0,0,0), ((1000,795),(1005,795),(1005,780), (1015,795),(1023,795),(1023, 775), (1018,775), (1018,790), (1008,775), (1000,775)))
                pygame.draw.line(KTHN, BLACK, [1000-68, 840], [1000, 840], 2)
                pygame.draw.line(KTHN, BLACK, [1000-68, 840], [1000-68, 837], 2)
                pygame.draw.line(KTHN, BLACK, [1000, 840], [1000, 837], 2)
                self.screen.blit(KTHN, (500,0))

                if self.Scene1 == True or self.CustScene == True :
                    L = self.WaypointsZoom()
                    for i in range(len(L)):
                        x, y = L[i][0], L[i][1]
                        pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)

                if Click == True:
                    self.screen.blit(perso, self.position_perso)

            '''Kista - South'''
            if self.map1 == 'KistaS' :
                KistaS = pygame.image.load('KistaS.png').convert()
                KistaS = pygame.transform.scale(KistaS,(1072,872))
                pygame.draw.polygon(KistaS,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                pygame.draw.polygon(KistaS,(0,0,0), ((1000,795),(1005,795),(1005,780), (1015,795),(1023,795),(1023, 775), (1018,775), (1018,790), (1008,775), (1000,775)))
                pygame.draw.line(KistaS, BLACK, [1000 - 68, 840], [1000, 840], 2)
                pygame.draw.line(KistaS, BLACK, [1000 - 68, 840], [1000 - 68, 837], 2)
                pygame.draw.line(KistaS, BLACK, [1000, 840], [1000, 837], 2)
                self.screen.blit(KistaS, (500,0))

                if self.Scene1 == True or self.CustScene == True :
                    L = self.WaypointsZoom()
                    for i in range(len(L)):
                        x, y = L[i][0], L[i][1]
                        pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)

                if Click == True:
                    '''  Displays the moving image '''
                    self.screen.blit(perso, self.position_perso)

            ''' KTH - South '''        
            if self.map1 == 'KTHS' :
                KTHS=pygame.transform.scale(pygame.image.load('KTHS.png').convert(), (1072,872))
                pygame.draw.polygon(KTHS,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                pygame.draw.polygon(KTHS,(0,0,0), ((1000,795),(1005,795),(1005,780), (1015,795),(1023,795),(1023, 775), (1018,775), (1018,790), (1008,775), (1000,775)))
                pygame.draw.line(KTHS, BLACK, [1000 - 68, 840], [1000, 840], 2)
                pygame.draw.line(KTHS, BLACK, [1000 - 68, 840], [1000 - 68, 837], 2)
                pygame.draw.line(KTHS, BLACK, [1000, 840], [1000, 837], 2)
                self.screen.blit(KTHS, (500,0))

                if self.Scene1 == True or self.CustScene == True :
                    L = self.WaypointsZoom()
                    for i in range(len(L)):
                        x, y = L[i][0], L[i][1]
                        pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)

                if Click == True :
                    '''  Displays the moving image '''
                    self.screen.blit(perso, self.position_perso)

            ''' KTH '''
            if self.map1 == 'KTHMap' :
                KTHMap = pygame.transform.scale(pygame.image.load('mapKTH.png').convert(), (1072,872))
                pygame.draw.polygon(KTHMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                pygame.draw.polygon(KTHMap,(0,0,0), ((1000,795),(1005,795),(1005,780), (1015,795),(1023,795),(1023, 775), (1018,775), (1018,790), (1008,775), (1000,775)))
                pygame.draw.line(KTHMap, BLACK, [1000 - 68, 840], [1000, 840], 2)
                pygame.draw.line(KTHMap, BLACK, [1000 - 68, 840], [1000 - 68, 837], 2)
                pygame.draw.line(KTHMap, BLACK, [1000, 840], [1000, 837], 2)
                self.screen.blit(KTHMap, (500,0))

                if Click == True :
                    '''  Displays the moving image '''
                    self.screen.blit(perso, self.position_perso)

                ''' Display waypoints of scenario 1'''
                if self.Scene1 == True :
                    print('ENTER')
                    for i in range (len(Tableau1)) :
                        x, y = Tableau1[i][0], Tableau1[i][1]
                        pygame.draw.circle(self.screen, ORANGE, (x, y), 5, 0)
                        print(i)
                        pygame.display.update()

                ''' Display waypoints of custumized scenario '''
                if self.CustScene == True :
                        #print(Tableau3)
                        for i in range (len(Tableau3)):
                            ''' Parcourt  '''
                            x, y = Tableau3[i][0], Tableau3[i][1]
                            pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)
                            pygame.display.update()

                        if self.map1 == 'KistaMap' :
                            pygame.draw.circle(self.screen, GREEN, (xS_Kista,yS_Kista), 5, 0)
                        if self.map1 == 'KTHMap' :
                            pygame.draw.circle(self.screen, GREEN, (xS_KTH, yS_KTH), 5, 0)

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
                    self.screen.blit(perso, self.position_perso)

                ''' Display waypoints of scenario 1'''
                if self.Scene1 == True :
                    print('ENTER')
                    for i in range (len(Tableau1)) :
                        x, y = Tableau1[i][0], Tableau1[i][1]
                        pygame.draw.circle(self.screen, ORANGE, (x, y), 5, 0)
                        print(i)
                        pygame.display.update()

                ''' Display waypoints of custumized scenario '''
                if self.CustScene == True :
                        #print(Tableau3)
                        for i in range (len(Tableau3)):
                            ''' Parcourt  '''
                            x, y = Tableau3[i][0], Tableau3[i][1]
                            pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)
                            pygame.display.update()

                        if self.map1 == 'KistaMap' :
                            pygame.draw.circle(self.screen, GREEN, (xS_Kista,yS_Kista), 5, 0)
                        if self.map1 == 'KTHMap' :
                            pygame.draw.circle(self.screen, GREEN, (xS_KTH, yS_KTH), 5, 0)

            if event.type == MOUSEBUTTONDOWN:
                print('MOUSEBUTTONDOWN', event.pos[1])


           

            ''' Display the quality of the GPS data '''
            '''if GPSQuali=='fix':
                green_square=pygame.image.load('green_square.png').convert()
                self.screen.blit(self.fond, (?, ?)

            elif GPSQuali=='single':
                red_square=pygame.image.load('red_square.png').convert()
                self.screen.blit(self.fond, (?, ?))

            elif GPSQuali=='float':
                orange_square=pygame.image.load('orange_square.png').convert()
                self.screen.blit(self.fond, (?, ?))'''
            orange_square=pygame.transform.scale(pygame.image.load('orange_square.png').convert(), (100,50))
            self.fond.blit(orange_square, (315,325))

            ''' Obstacle detection '''
            if Obstacle == 'True':
                self.obt_detec = 'True'
            else:
                self.obt_detec = 'False'


            self.update_textes()
            for text in self.textes:

                self.display_text(text[0], text[1], text[2],
                                        text[3], text[4])

            # Ajout du fond dans la fenêtre
            self.screen.blit(self.fond, (0, 0))

            t = clock.get_time() / 1000
           
            #Calcul de la distance
            #On stocke l'ancienne position
            if variablePosition % 2 == 0 and state_x != None and state_y != None :
            	StockPosition[0][0], StockPosition[0][1] = self.position_perso.centerx, self.position_perso.centery
            	variablePosition += 1
            elif variablePosition % 2 == 1 and state_x != None and state_y != None :
            	StockPosition[1][0], StockPosition[1][1] = self.position_perso.centerx, self.position_perso.centery
            	variablePosition += 1

            euclid = ((StockPosition[0][0] - StockPosition[1][0]) * EchelleMPix)**2 + ((StockPosition[0][1] - StockPosition[1][1]) * EchelleMPix)**2
            d = sqrt(euclid)
            if t != 0 :
            	vitesse = d / t
            	vitesse = int(vitesse * 100) / 100
            	#Mise a jour de la valeur de la vitesse
            	#print(V)
            	self.level = vitesse

            #Update the status
            
            if state_x!=None and state_y!=None :
            	self.status = 'Running'
            if  StockPosition[0][0]-StockPosition[1][0]==0 and StockPosition[0][1]-StockPosition[1][1]==0:
            	self.status = 'No subscriber'

            # Actualisation de l'affichage
            pygame.display.update()
            # 10 fps
            clock.tick(5)




def start():
    print("start")

def gamequit():
    print("Quit")
    pygame.quit()
    sys.exit()



def waypointsPublish():
#Starts publishing 10 waypoints on the topic "waypoints". It publishes once at the mousebutton click Tableau1=[[Xi,Yi]i], waypoints coordinates
    point1, point2, point3, point4, point5, point6, point7, point8, point9, point10 =Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose()
    Array = PoseArray()

    point1.position.x, point1.position.y = Tableau1[0][0], Tableau1[0][1]
    point2.position.x, point2.position.y = Tableau1[1][0], Tableau1[1][1]
    point3.position.x, point3.position.y = Tableau1[2][0], Tableau1[2][1]
    point4.position.x, point4.position.y = Tableau1[3][0], Tableau1[3][1]
    point5.position.x, point5.position.y = Tableau1[4][0], Tableau1[4][1]
    point6.position.x, point6.position.y = Tableau1[5][0], Tableau1[5][1]
    point7.position.x, point7.position.y = Tableau1[6][0], Tableau1[6][1]
    point8.position.x, point8.position.y = Tableau1[7][0], Tableau1[7][1]
    point9.position.x, point9.position.y = Tableau1[8][0], Tableau1[8][1]
    point10.position.x, point10.position.y = Tableau1[9][0], Tableau1[9][1]

    Array.poses.extend((point1, point2, point3, point4, point5, point6, point7, point8, point9, point10))
    
    rospy.loginfo(Array)
    goalPublisher.publish(Array)


    #rate=rospy.Rate(10)
    #rate.sleep()
def waypoints_personalized():
    '''Publishes the waypoints coming from the personalized scenario
    Assumes that the max number of waypoints is 10. There is an undetermined number of waypoints which are treated with the list Stock '''
    
    print('Waypoints Personalized')
    #print(Tableau2)
    NbrePoints = len(Tableau2)
    point1, point2, point3, point4, point5, point6, point7, point8, point9, point10 = Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose()
    Array = PoseArray()
    Stock = [point1, point2, point3, point4, point5, point6, point7, point8, point9, point10]

    for i in range (NbrePoints):
        Stock[i].position.x, Stock[i].position.y = Tableau2[i][0], Tableau2[i][1]
        
    Array.poses.extend((Stock[i]) for i in range(NbrePoints))
        
    rospy.loginfo(Array)
    goalPublisher.publish(Array)

def waypointsPublish2():
    print 'Scenario 2'

if __name__ == '__main__':
    game = Game()
    game.infinite_loop()

   