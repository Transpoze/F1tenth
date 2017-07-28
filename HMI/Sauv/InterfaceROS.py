#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# server_gui.py

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


#Initialize ROS node
rospy.init_node('GUIControl', anonymous=True)

#Initialize global variables
V=None
X=None
Y=None
state_x=None
state_y=None
Tableau2=[]

#Define the subscriber to the velocity
def callbackVel(msg):
    global V
    #rospy.loginfo("Received a /cmd_vel message!")
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s", Twist.linear)
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    #rospy.loginfo("Conversion : x lineaire : [%f], x converti: [%f]" %(msg.linear.x, msg.linear.x*10))
    V=msg.linear.x
    
Subs=rospy.Subscriber("/turtle1/cmd_vel", Twist, callbackVel)

#Define the subscriber to the position
def callbackPose(msg):
    global X, Y
    #rospy.loginfo("Received a /Pose message !!")
    #rospy.loginfo("Position de la tortue : [%f, %f]" %(msg.x, msg.y) )
    X=msg.x
    Y=msg.y

SubsPose=rospy.Subscriber("/turtle1/pose", Pos, callbackPose)

#Define the subscriber to the position
def callbackCar_State(msg):
    global state_x, state_y
    #rospy.loginfo("Received a /Pose message !!")
    #rospy.loginfo("Position de la tortue : [%f, %f]" %(msg.x, msg.y) )
    state_x=msg.pose.pose.position.x
    state_y=msg.pose.pose.position.y

SubsCar_State=rospy.Subscriber("/car_state_topic", Odometry, callbackCar_State)


#Initialize publisher of the waypoints
goalPublisher=rospy.Publisher('waypoints', PoseArray, queue_size=10)

#Date 
date=datetime.datetime.now()


#Waypoints coordinates
Tableau1=[[995, 789], [995, 670], [995, 550],[995, 802], [995, 728], [995, 200],[995, 127],[995, 186],[995, 217],[995, 42]]
print(len(Tableau1))

class Button:
    '''Ajout d'un bouton avec un texte sur img
    Astuce: ajouter des espaces dans les textes pour avoir une même largeur
    de boutons
    dx, dy décalage du bouton par rapport au centre
    action si click
    Texte noir
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
        self.position_perso=(600,600)
        self.status = 'Waiting to begin'
        self.scale=1
        # Définition de la police
        self.big = pygame.font.SysFont('freesans', 48)
        self.small = pygame.font.SysFont('freesans', 36)
        self.mediumsmall = pygame.font.SysFont('freesans',28)
        self.verysmall = pygame.font.SysFont('freesans', 10)
        self.map = 'KTHMap' #defaultValue

        self.create_fond()
        self.create_button()
        self.zoom = 1  #1 si pas Zoom 2 si Zoom

    def update_textes(self):
        self.textes = [ ["Autonomous Driving with F1/10", ORANGE, self.small, 0, 50],
                        ["Speed :", BLACK, self.small, -110, 150],
                        [str(self.level), BLACK, self.small, 0, 150],
                        ["Status :", BLACK, self.small, -110, 250],
                        [str(self.status), BLACK, self.small, 70, 250],
                        ["Destination :", BLACK, self.small, -70, 350],
                        ["Maps :", BLACK, self.small, -110, 450],
                        ["KTH Campus, Stockholm", BLACK, self.verysmall, 160, 840],
                        ["OpenStreetMap ", BLACK, self.verysmall, 142, 850],
                        [str(date), BLACK, self.verysmall, 172, 861],
                        ]

    def create_fond(self):
        # Image de la taille de la fenêtre
        self.fond = pygame.Surface((500, 872))
        # En bleu
        self.fond.fill(WHITE)

    def create_button(self):
        self.reset_button = Button(self.fond, "   Reset   ", MARRON, self.small, -40, 853)
        self.quit_button  = Button(self.fond, "   Quit   ", RED, self.small, -180, 853)
        self.Scenario1_button = Button(self.fond, "  Scenario 1  ", MARRON2, self.small, -80, 550)
        self.Scenario2_button=Button(self.fond,"  Scenario 2  ", MARRON2, self.small, 130,550)
        self.Kista_button = Button(self.fond, "  Kista  ", BEIGE, self.small, 130, 450)
        self.KTH_button = Button(self.fond, "  KTH  ", BEIGE, self.small, 10, 450)
        self.PersoScenario_button = Button(self.fond, "  Customized Scenario  ", MARRON2, self.small, 20, 595)

    def display_text(self, text, color, font, dx, dy):
        '''Ajout d'un texte sur fond. Décalage dx, dy par rapport au centre.
        '''
        mytext = font.render(text, True, color)  # True pour antialiasing
        textpos = mytext.get_rect()
        textpos.centerx = self.fond.get_rect().centerx + dx
        textpos.centery = dy
        self.fond.blit(mytext, textpos)

    def reset(self):
    	KistaMap=pygame.image.load('mapKista.png').convert()
        KTHMap=pygame.transform.scale(pygame.image.load('mapKTH.png').convert(), (1072,872))
        if self.map=='KTHMap':
        	pygame.draw.polygon(KTHMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
        	pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000, 840], 2)
        	pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
        	pygame.draw.line(KTHMap, BLACK, [1000, 840], [1000, 837], 2)
        	self.screen.blit(KTHMap, (500,0))
        	pygame.display.update()
        elif self.map=='KistaMap':
        	pygame.draw.polygon(KistaMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
        	pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000, 840], 2)
        	pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
        	pygame.draw.line(KistaMap, BLACK, [1000, 840], [1000, 837], 2)
        	self.screen.blit(KistaMap, (500,0))
        	pygame.display.update()
        self.zoom=1
    	print("reset")

    def Display_KistaMap(self):
    	self.zoom=1
        KistaMap=pygame.image.load('mapKista.png').convert()
        pygame.draw.polygon(KistaMap,BLACK, ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
        pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000, 840], 2)
        pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
        pygame.draw.line(KistaMap, BLACK, [1000, 840], [1000, 837], 2)
        self.screen.blit(KistaMap, (500,0))
        self.map='KistaMap'
        pygame.display.update()

    def Display_KTHMap(self):
    	self.zoom=1
        KTHMap=pygame.image.load('mapKTH.png').convert()
        KTHMap=pygame.transform.scale(pygame.image.load('mapKTH.png').convert(), (1072,872))
        pygame.draw.polygon(KTHMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
        pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000, 840], 2)
        pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
        pygame.draw.line(KTHMap, BLACK, [1000, 840], [1000, 837], 2)
        self.screen.blit(KTHMap, (500,0))
        self.map='KTHMap'
        pygame.display.update()


    def Scenario1(self):
        #Displays the waypoints of scenario 1
        
        for i in range (len(Tableau1)):
            x,y=Tableau1[i][0], Tableau1[i][1]
            pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)
            print(i)
        waypointsPublish()


    def CreatePersonalizedScenario(self):
        #Create waypoints by mouse clicking on the map;
        #Press Return to stop the selection 
        #Add restriction if the click is not on the map
        #x,y : position relative to the left corner of the display
       
        continuer=1
        while continuer:
            for event in pygame.event.get():
                print('Get a event msg')
                if event.type==MOUSEBUTTONDOWN and event.button==1:
                    print('Get a mousebutton msg')
                    x,y=pygame.mouse.get_pos()[0],pygame.mouse.get_pos()[1]
                    P=[x,y]
                    Tableau2.extend([P])
                    pygame.draw.circle(self.screen, ORANGE, (x,y), 5, 0)
                    pygame.display.update()
                print(Tableau2)
                if event.type==KEYDOWN :
                    if event.key==K_RETURN:
                        waypoints_personalized()
                        continuer=0
        


    #def ConvGPSScale1(self):



    def infinite_loop(self):

    	#DONNEES
    	    #Caracteristiques de la carte #faire un input
    	#KTHMap
    	LongitMin_KTH=18.06208
    	LongitMax_KTH=18.07555
    	LatitMin_KTH=59.34838
    	LatitMax_KTH=59.35397
    	fenetreWidth=1071
    	fenetreHeight=872
    	EchelleLongit_KTH=((LongitMax-LongitMin)/fenetreWidth)
    	EchelleLatit_KTH=((LatitMax-LatitMin)/fenetreHeight)

    	#KistaMap
    	LongitMin_Kista=17.95017
    	LongitMax_Kista=17.95354
    	LatitMin_Kista=59.40374
    	LatitMax_Kista=59.40517
    	fenetreWidth=1071
    	fenetreHeight=872
    	EchelleLongit_Kista=((LongitMax-LongitMin)/fenetreWidth)
    	EchelleLatit_Kista=((LatitMax-LatitMin)/fenetreHeight)

    	#10metres=68pixels 
    	EchelleMPix=10/68
    	print(EchelleMPix)

        #Starting Point arbitraire
        #Il en faut 2 : pour Kista et pour KTH
        #KTH
        lS_KTH,LS_KTH=59.349468,18.069826
        lS_Kista, LS_Kista=59.404519,17.951890


        

        #CONVERSION LATITUDE LONGITUDE > PIXELS
       #xs et yS : starting point arbitraire
        xS_KTH, yS_KTH=int(((lS_KTH-LatitMin_KTH)/EchelleLatit_KTH)+500),int(((LongitMax_KTH-LS_KTH)/EchelleLongit_KTH))
        xS_Kista, yS_Kista=int(((lS_Kista-LatitMin)/EchelleLatit)+500),int(((LongitMax-LS_Kista)/EchelleLongit))




        while self.loop:
            self.create_fond()

            #Mise a jour de la valeur de la vitesse
            #print(V)
            self.level=V

            #Update the status
            self.status="No subscriber"

            #Check car_state_topic subscriber :
            #print('state_x', state_x, 'state_y', state_y)
            
            #Load the moving point picture
            perso=pygame.image.load('square.png').convert()
            self.position_perso=perso.get_rect()

            # Boutons
            self.reset_button.display_button(self.fond)
            self.quit_button.display_button(self.fond)
            self.Scenario1_button.display_button(self.fond)
            self.Scenario2_button.display_button(self.fond)
            self.Kista_button.display_button(self.fond)
            self.KTH_button.display_button(self.fond)
            self.PersoScenario_button.display_button(self.fond)

            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.reset_button.update_button(self.fond, action=self.reset)
                    self.quit_button.update_button(self.fond, action=gamequit)
                    self.Scenario1_button.update_button(self.fond, action=self.Scenario1)
                    self.Scenario2_button.update_button(self.fond, action=waypointsPublish2)
                    self.Kista_button.update_button(self.fond, action=self.Display_KistaMap)
                    self.KTH_button.update_button(self.fond, action=self.Display_KTHMap)
                    self.PersoScenario_button.update_button(self.fond, action=self.CreatePersonalizedScenario)

                    #DE-ZOOM
                    if event.button==3 and event.pos[0]>500:
                    	self.zoom=2
                    	print('bouton droit')
                    	if self.map=='KTHMap':
                    		KTHMap=pygame.transform.scale(pygame.image.load('mapKTH.png').convert(), (1072,872))
                    		pygame.draw.polygon(KTHMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                    		pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000, 840], 2)
                    		pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
                    		pygame.draw.line(KTHMap, BLACK, [1000, 840], [1000, 837], 2)
                    		self.screen.blit(KTHMap, (500,0))
                    	elif self.map=='KistaMap':
                    		KistaMap=pygame.image.load('mapKista.png').convert()
                    		pygame.draw.polygon(KistaMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                    		pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000, 840], 2)
                    		pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
                    		pygame.draw.line(KistaMap, BLACK, [1000, 840], [1000, 837], 2)
                    		self.screen.blit(KistaMap, (500,0))


                    #pos[0]=x et pos[1]=y
                    #ZOOM NORD
                #KistaMap=pygame.image.load('mapKista.png').convert()
                #KTHMap=pygame.image.load('mapKTH.png').convert()
                if event.type==MOUSEBUTTONDOWN and event.button==1 and event.pos[0]>500 and event.pos[1]<436:
                	self.zoom=1
                	if self.map=='KistaMap':
						#KistaN=pygame.image.load('KistaN.png').convert()
            			KistaN=pygame.transform.scale(pygame.image.load('KistaN.png').convert(),(1072,872))
            			pygame.draw.polygon(KistaN,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800