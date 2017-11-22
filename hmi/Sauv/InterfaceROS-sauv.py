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


#Initialize ROS node
rospy.init_node('GUIControl', anonymous=True)

#Initialize global variables
V=None
X=None
Y=None
state_x=None
state_y=None
ori=None
Tableau2=[]
Click=False

#Pour le calcul de la vitesse, intialisation :
StockPosition=[[0,0],[0,0]]

def constrainAngle(x):
	return x%2*np.pi
def convertRad2Deg(x):
	return(180*x)/np.pi

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
    global state_x, state_y,ori
    #rospy.loginfo("Received a simulation message !!")
    #rospy.loginfo("Position en rospy : [%f, %f]" %(msg.pose.pose.position.x, msg.pose.pose.position.y) )
    state_x=msg.pose.pose.position.x
    state_y=msg.pose.pose.position.y
    ori=msg.pose.pose.orientation.w
    ori=constrainAngle(ori)
    ori=convertRad2Deg(ori)
    rospy.loginfo("recu orientation: [%f]" %(ori))
    rospy.loginfo("recu position x: [%f]" %(state_x))



SubsCar_State=rospy.Subscriber("/car_state_topic", Odometry, callbackCar_State)
#SubsCar_State=rospy.Subscriber("qualisys/F1Tenth", Subject, callbackCar_State)



#Initialize publisher of the waypoints
goalPublisher=rospy.Publisher('waypoints', PoseArray, queue_size=10)

#Date 
date=datetime.datetime.now()


#Waypoints coordinates
Tableau1=[[995, 789], [995, 670], [995, 550],[995, 802], [995, 728], [995, 200],[995, 127],[995, 186],[995, 217],[995, 42]]


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
        self.map = 'KistaMap' #defaultValue

        self.create_fond()
        self.create_button()
        self.zoom = 1  #1 si pas Zoom 2 si Zoom

    def update_textes(self):
        self.textes = [ ["Autonomous Driving with F1/10", ORANGE, self.small, 0, 50],
                        ["Speed (m/s) :", BLACK, self.small, -65, 150],
                        [str(self.level), BLACK, self.small, 90, 150],
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
        self.reset_button = Button(self.fond, "   Reset   ", YELLOW, self.small, -40, 853)
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
        if self.map=='KTHS' or self.map=='KTHN' or self.map=='KTHMap':
        	pygame.draw.polygon(KTHMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
        	pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000, 840], 2)
        	pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
        	pygame.draw.line(KTHMap, BLACK, [1000, 840], [1000, 837], 2)
        	self.screen.blit(KTHMap, (500,0))
        	pygame.display.update()
        	self.map='KTHMap'
        elif self.map=='KistaN' or self.map=='KistaS' or self.map=='KistaMap':
        	pygame.draw.polygon(KistaMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
        	pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000, 840], 2)
        	pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
        	pygame.draw.line(KistaMap, BLACK, [1000, 840], [1000, 837], 2)
        	self.screen.blit(KistaMap, (500,0))
        	pygame.display.update()
        	self.map='KistaMap'
        self.zoom=1
    	print("reset")

    def Display_KistaMap(self):
    	self.zoom=1
    	global Click
    	Click=True
        KistaMap=pygame.image.load('mapKista.png').convert()
        pygame.draw.polygon(KistaMap,BLACK, ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
        pygame.draw.polygon(KistaMap,BLACK, ((1030,815),(1032,815),(1032,805), (1030,805),(1027,811),(1027, 811), (1028,807)))
        pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000, 840], 2)
        pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
        pygame.draw.line(KistaMap, BLACK, [1000, 840], [1000, 837], 2)
        self.screen.blit(KistaMap, (500,0))
        self.map='KistaMap'
        pygame.display.update()

    def Display_KTHMap(self):
    	self.zoom=1
    	global Click
    	Click=True
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

        #First, let's reset the map in case waypoints were arleady there
        self.reset()
        
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

        #First, let's reset the map in case waypoints were arleady there
        self.reset()

        info_custoScenario=pygame.image.load('HMI-Info-CustoScenario.png').convert()
        self.screen.blit(info_custoScenario, (550,780))
        pygame.display.update()
       
        continuer=1
        while continuer:
            for event in pygame.event.get():
                print('Get a event msg')
                if event.type==MOUSEBUTTONDOWN and event.button==1:
                    print('Get a mousebutton msg')
                    x,y=pygame.mouse.get_pos()[0],pygame.mouse.get_pos()[1]
                    if x>500 :
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
    	LongitMin_KTH=18.06783
    	LongitMax_KTH=18.07120
    	LatitMin_KTH=59.34886
    	LatitMax_KTH=59.35029
    	fenetreWidth=1071
    	fenetreHeight=872
    	EchelleLongit_KTH=((LongitMax_KTH-LongitMin_KTH)/fenetreWidth)
    	EchelleLatit_KTH=((LatitMax_KTH-LatitMin_KTH)/fenetreHeight)

    	#KistaMap
    	LongitMin_Kista=17.95017
    	LongitMax_Kista=17.95354
    	LatitMin_Kista=59.40374
    	LatitMax_Kista=59.40517
    	fenetreWidth=1071
    	fenetreHeight=872
    	EchelleLongit_Kista=((LongitMax_Kista-LongitMin_Kista)/fenetreWidth)
    	EchelleLatit_Kista=((LatitMax_Kista-LatitMin_Kista)/fenetreHeight)

    	#10metres=68pixels 
    	EchelleMPix=10/68

        #Starting Point arbitraire
        #Il en faut 2 : pour Kista et pour KTH
        #KTH
        #lS_KTH,LS_KTH=59.349603,18.069659
        lS_KTH,LS_KTH=59.349451,18.069778
        #lS_Kista, LS_Kista=59.404503,17.951875
        lS_Kista, LS_Kista=59.404227,17.952103

        variablePosition=0
        print('Subscrib orient :', ori)

        #CONVERSION LATITUDE LONGITUDE > PIXELS
        #xs et yS : starting point arbitraire
        yS_KTH, xS_KTH=int((LatitMax_KTH-lS_KTH)/EchelleLatit_KTH),int((((LS_KTH-LongitMin_KTH)/EchelleLongit_KTH))+500)
        yS_Kista, xS_Kista=int((LatitMax_Kista-lS_Kista)/EchelleLatit_Kista),int((((LS_Kista-LongitMin_Kista)/EchelleLongit_Kista))+500)


        ChooseMap=pygame.image.load('ChooseMap.png').convert()
        self.screen.blit(ChooseMap, (550,780))
        pygame.display.update()
      
        while self.loop:
            self.create_fond()

            #Check car_state_topic subscriber :
            print('state_x', state_x, 'state_y', state_y)
            if ori!=None:
           		print('Orientation =', ori)

            
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
                if event.type==pygame.MOUSEBUTTONDOWN and event.button==3 and event.pos[0]>500:
                    	self.zoom=1
                    	print('bouton droit')
                    	if self.map=='KTHS' or self.map=='KTHN':
                    		KTHMap=pygame.transform.scale(pygame.image.load('mapKTH.png').convert(), (1072,872))
                    		pygame.draw.polygon(KTHMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                    		pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000, 840], 2)
                    		pygame.draw.line(KTHMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
                    		pygame.draw.line(KTHMap, BLACK, [1000, 840], [1000, 837], 2)
                    		self.screen.blit(KTHMap, (500,0))
                    		self.map='KTHMap'

                    		if Click==True:
            					self.screen.blit(perso, self.position_perso)

                    	elif self.map=='KistaN' or self.map=='KistaS':
                    		KistaMap=pygame.image.load('mapKista.png').convert()
                    		pygame.draw.polygon(KistaMap,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
                    		pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000, 840], 2)
                    		pygame.draw.line(KistaMap, BLACK, [1000-68, 840], [1000-68, 837], 2)
                    		pygame.draw.line(KistaMap, BLACK, [1000, 840], [1000, 837], 2)
                    		self.screen.blit(KistaMap, (500,0))
                    		self.map='KistaMap'

                    		if Click==True:
            					self.screen.blit(perso, self.position_perso)


                    #pos[0]=x et pos[1]=y
                    #ZOOM NORD
                #KistaMap=pygame.image.load('mapKista.png').convert()
                #KTHMap=pygame.image.load('mapKTH.png').convert()
                if event.type==MOUSEBUTTONDOWN and event.button==1 and event.pos[0]>500 and event.pos[1]<436:
                	self.zoom=2
                	if self.map=='KistaMap' or self.map=='KistaS':
						#KistaN=pygame.image.load('KistaN.png').convert()
            			KistaN=pygame.transform.scale(pygame.image.load('KistaN.png').convert(),(1072,872))
            			pygame.draw.circle(self.screen, ORANGE, ( xS_Kista, yS_Kista), 5, 0)
            			pygame.draw.polygon(KistaN,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
            			pygame.draw.line(KistaN, BLACK, [1000-68, 840], [1000, 840], 2)
            			pygame.draw.line(KistaN, BLACK, [1000-68, 840], [1000-68, 837], 2)
            			pygame.draw.line(KistaN, BLACK, [1000, 840], [1000, 837], 2)
            			self.screen.blit(KistaN, (500,0))
            			self.map='KistaN'

            			if Click==True:
            				self.screen.blit(perso, self.position_perso)

            		elif self.map=='KTHMap' or self.map=='KTHS':
            			#KTHN=pygame.image.load('KTHN.png').convert()
            			self.map='KTHN'
            			KTHN=pygame.transform.scale(pygame.image.load('KTHN.png').convert(), (1072,872))
            			pygame.draw.circle(self.screen, ORANGE, ( xS_KTH, yS_KTH), 5, 0)
            			pygame.draw.polygon(KTHN,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
            			pygame.draw.line(KTHN, BLACK, [1000-68, 840], [1000, 840], 2)
            			pygame.draw.line(KTHN, BLACK, [1000-68, 840], [1000-68, 837], 2)
            			pygame.draw.line(KTHN, BLACK, [1000, 840], [1000, 837], 2)
            			self.screen.blit(KTHN, (500,0))

            			if Click==True:
            				self.screen.blit(perso, self.position_perso)

     
            			#ZOOM SUD
            	if event.type==MOUSEBUTTONDOWN and event.button==1 and event.pos[0]>500 and event.pos[1]>436:
            		self.zoom=2
            		if self.map=='KistaMap' or self.map=='KistaN':
            			self.map='KistaS'
            			KistaS=pygame.image.load('KistaS.png').convert()
            			KistaS=pygame.transform.scale(KistaS,(1072,872))
            			pygame.draw.polygon(KistaS,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
            			pygame.draw.line(KistaS, BLACK, [1000-68, 840], [1000, 840], 2)
            			pygame.draw.line(KistaS, BLACK, [1000-68, 840], [1000-68, 837], 2)
            			pygame.draw.line(KistaS, BLACK, [1000, 840], [1000, 837], 2)
            			self.screen.blit(KistaS, (500,0))

            			if Click==True:
            				self.screen.blit(perso, self.position_perso)

            		elif self.map=='KTHMap' or self.map=='KTHN':
            			self.map='KTHS'
            			KTHS=pygame.transform.scale(pygame.image.load('KTHS.png').convert(), (1072,872))
            			pygame.draw.polygon(KTHS,(0,0,0), ((1000,810),(1008,810),(1008,840), (1012,840),(1012,810),(1020, 810), (1010,800)))
            			pygame.draw.line(KTHS, BLACK, [1000-68, 840], [1000, 840], 2)
            			pygame.draw.line(KTHS, BLACK, [1000-68, 840], [1000-68, 837], 2)
            			pygame.draw.line(KTHS, BLACK, [1000, 840], [1000, 837], 2)
            			self.screen.blit(KTHS, (500,0))

            			
                    	

           	
           	#Determination des positions du point au cours du temps a partir de la simulation de Sud (position relative en m)
           	#CONVERSION METRES > PIXELS
        	#delta_xp et delta_yp sont les positions relatives donnees par la simulation de Sud en m
        	yS_KTH, xS_KTH=int((LatitMax_KTH-lS_KTH)/EchelleLatit_KTH),int((((LS_KTH-LongitMin_KTH)/EchelleLongit_KTH))+500)
            yS_Kista, xS_Kista=int((LatitMax_Kista-lS_Kista)/EchelleLatit_Kista), int((((LS_Kista-LongitMin_Kista)/EchelleLongit_Kista))+500)
		
            if self.zoom==1 and xS_Kista!=None and yS_Kista!=None and state_x!=None and state_y!=None and self.map=='KistaMap':
            		delta_xp,delta_yp=(state_x*2)/EchelleMPix,(state_y*2)/EchelleMPix
            		#print('state_x', state_x, 'state_y', state_y)
            		#print('delta_xp', delta_xp, 'delta_yp', delta_yp)
            		pygame.draw.circle(self.screen, ORANGE, ( xS_Kista, yS_Kista), 5, 0)
            		x, y = int(xS_Kista+delta_xp), int(yS_Kista+delta_yp)
            		self.position_perso.centerx=x
            		self.position_perso.centery=y
            		print(self.position_perso.centerx, self.position_perso.centery)

            if self.zoom==1 and xS_KTH!=None and yS_KTH!=None and state_x!=None and state_y!=None and self.map=='KTHMap':
            		delta_xp,delta_yp=(state_x*2)/EchelleMPix,(state_y*2)/EchelleMPix
            		#print('state_x', state_x, 'state_y', state_y)
            		#print('x Kista', xS_Kista, 'y Kista', yS_Kista)
            		pygame.draw.circle(self.screen, ORANGE, ( xS_KTH, yS_KTH), 5, 0)
            		x, y = int(xS_KTH+delta_xp), int(yS_KTH+delta_yp)
            		self.position_perso.centerx=x
            		self.position_perso.centery=y
            		#print('delta_xp', delta_xp, 'delta_yp', delta_yp)
            		print(self.position_perso.centerx, self.position_perso.centery)

            
            if self.zoom==2 and state_x!=None and state_y!=None and self.map=='KTHN':
            	#KTHN
            	#bottomright=(1045,1349) resized at (1073,739)
            	#Longueur, largeur = 549,521 (voir Paint)
            	#points donnes en echelle 1:
            	print('Entre Zoom KTH N',x,y)
            	delta_xp,delta_yp=(state_x*2)/EchelleMPix,(state_y*2)/EchelleMPix
            	a, b =1071/301, -2837
            	c, d = 872/286, -875
            	if x>937 and x<1240 and y>286 and y<574:
            		x, y = int(xS_KTH+delta_xp), int(yS_KTH+delta_yp)
            		x2, y2 = int(a*x+b), int(c*y+d)
            		print('Zoom KTHN',x,y,x2,y2)
            		self.position_perso.centerx=x2
            		self.position_perso.centery=y2

            if self.zoom==2 and state_x!=None and state_y!=None and self.map=='KTHS':
            	#KistaN
            	#topleft=(1345,819)
            	#Longueur, largeur = 433,561 (voir Paint)
            	#points donnes en echelle 1:
            	print('Entre Zoom Kista N',x,y)
            	delta_xp,delta_yp=(state_x*2)/EchelleMPix,(state_y*2)/EchelleMPix
            	a, b =1071/237, -4018
            	c, d = 872/307, -1275
            	if x>999 and x<1238 and y>448 and y<757:
            		x, y = int(xS_KTH+delta_xp), int(yS_KTH+delta_yp)
            		x2, y2 = int(a*x+b), int(c*y+d)
            		print('Zoom KTHS',x,y,x2,y2)
            		self.position_perso.centerx=x2
            		self.position_perso.centery=y2

            if self.zoom==2 and state_x!=None and state_y!=None and self.map=='KistaN':
            	#KistaN
            	#topleft=(284,156)
            	#Longueur, largeur = 399,417 (voir Paint)
            	#points donnes en echelle 1:
            	print('Entre Zoom Kista N',x,y)
            	delta_xp,delta_yp=(state_x*2)/EchelleMPix,(state_y*2)/EchelleMPix
            	a, b =1071/399, -1604
            	c, d = 872/417, -326
            	if x>783 and x<1184 and y>156 and y<573:
            		x, y = int(xS_Kista+delta_xp), int(yS_Kista+delta_yp)
            		x2, y2 = int(a*x+b), int(c*y+d)
            		print('Zoom KistaN',x,y,x2,y2)
            		self.position_perso.centerx=x2
            		self.position_perso.centery=y2

            if self.zoom==2 and state_x!=None and state_y!=None and self.map=='KistaS':
            	x,y=int(xS_Kista+(state_x*2)/EchelleMPix), int(yS_Kista+(state_y*2)/EchelleMPix)
            	a,b=1071/414, -1859
            	c,d=872/419, -636
            	print('Entre Zoom Kista S',x,y)
            	if x<1227 and x>911 and y>305 and y<725:
            		x2,y2=int(a*x+b), int(c*y+d)
            		print('Zoom Kista Sud', x, y, x2, y2)
            		self.position_perso.centerx=x2
            		self.position_perso.centery=y2
 
            self.update_textes()
            for text in self.textes:

                self.display_text(text[0], text[1], text[2],
                                        text[3], text[4])

            # Ajout du fond dans la fenêtre
            self.screen.blit(self.fond, (0, 0))
            #pygame.draw.circle(self.screen, ORANGE, ( xS_KTH, yS_KTH), 5, 0)

            t=clock.get_time()/1000
           
            #Calcul de la distance
            #On stocke l'ancienne position
            if variablePosition%2==0 and state_x!=None and state_y!=None:
            	StockPosition[0][0], StockPosition[0][1]=x, y
            	variablePosition+=1
            elif variablePosition%2==1 and state_x!=None and state_y!=None :
            	StockPosition[1][0], StockPosition[1][1]=x, y
            	variablePosition+=1

            euclid = ((StockPosition[0][0]-StockPosition[1][0])*EchelleMPix)**2+((StockPosition[0][1]-StockPosition[1][1])*EchelleMPix)**2
            d=sqrt(euclid)
            if t!=0:
            	vitesse=d/t
            	vitesse=int(vitesse*100)/100
            	#Mise a jour de la valeur de la vitesse
            	#print(V)
            	self.level=vitesse

            #Update the status
            
            if state_x!=None and state_y!=None :
            	self.status = 'Running'
            if  StockPosition[0][0]-StockPosition[1][0]==0 and StockPosition[0][1]-StockPosition[1][1]==0:
            	self.status = 'No subscriber'
            	
            print(self.status, StockPosition[0][0]-StockPosition[1][0],StockPosition[0][1]-StockPosition[1][1])

            #Arrow
            if type(ori)==float:
            	fleche=pygame.image.load('fleche.jpg').convert()
            	fleche=pygame.transform.scale(fleche, (35,35))
            	#Rotation de l'image fleche
            	fleche=pygame.transform.rotate(fleche, ori)
            	fleche.set_colorkey((255,255,255))	
            	#self.screen.blit(fleche, self.position_perso)		

            
            	#print(self.position_perso)
            if Click==True:
            	self.screen.blit(perso, self.position_perso)

            
            # Actualisation de l'affichage
            pygame.display.update()
            # 10 fps
            clock.tick(10)




def start():
    print("start")

def gamequit():
    print("Quit")
    pygame.quit()
    sys.exit()



def waypointsPublish():
#Starts publishing 10 waypoints on the topic 'waypoints'
#It publishes once at the mousebutton click
#Tableau1=[[Xi,Yi]i], waypoints coordinates 
    point1, point2, point3, point4, point5, point6, point7, point8, point9, point10 =Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose()
    Array=PoseArray()

    point1.position.x, point1.position.y=Tableau1[0][0],Tableau1[0][1]
    point2.position.x, point2.position.y=Tableau1[1][0],Tableau1[1][1]
    point3.position.x, point3.position.y=Tableau1[2][0],Tableau1[2][1]
    point4.position.x, point4.position.y=Tableau1[3][0],Tableau1[3][1]
    point5.position.x, point5.position.y=Tableau1[4][0],Tableau1[4][1]
    point6.position.x, point6.position.y=Tableau1[5][0],Tableau1[5][1]
    point7.position.x, point7.position.y=Tableau1[6][0],Tableau1[6][1]
    point8.position.x, point8.position.y=Tableau1[7][0],Tableau1[7][1]
    point9.position.x, point9.position.y=Tableau1[8][0],Tableau1[8][1]
    point10.position.x, point10.position.y=Tableau1[9][0],Tableau1[9][1]

    Array.poses.extend((point1, point2, point3, point4, point5, point6, point7, point8, point9, point10))
    
    rospy.loginfo(Array)
    goalPublisher.publish(Array)


    #rate=rospy.Rate(10)
    #rate.sleep()
def waypoints_personalized():
    #Publishes the waypoints coming from the personalized scenario
    #Assumes that the max number of waypoints is 10. There is an undetermined number of waypoints which are treated with the list Stock.
    
    print('Waypoints Personalized')
    NbrePoints=len(Tableau2)
    point1, point2, point3, point4, point5, point6, point7, point8, point9, point10 =Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose()
    Array=PoseArray()
    Stock=[point1, point2, point3, point4, point5, point6, point7, point8, point9, point10]

    for i in range (NbrePoints):
        Stock[i].position.x,Stock[i].position.y=Tableau2[i][0],Tableau2[i][1]
        
    Array.poses.extend((Stock[i]) for i in range(NbrePoints))
        
    rospy.loginfo(Array)
    goalPublisher.publish(Array)

def waypointsPublish2():
    print 'Scenario 2'

if __name__ == '__main__':
    game = Game()
    game.infinite_loop()

   