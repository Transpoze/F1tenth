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

import pygame
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3


pygame.init()
clock = pygame.time.Clock()

BLACK = 0, 0, 0
WHITE = 255, 255, 255
CIEL = 0, 200, 255
RED = 255, 0, 0
ORANGE = 255, 100, 0
GREEN = 0, 255, 0


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
    def __init__(self):
        self.screen = pygame.display.set_mode((1571, 872))
        self.level = 1
        self.loop = True

        # Définition de la police
        self.big = pygame.font.SysFont('freesans', 48)
        self.small = pygame.font.SysFont('freesans', 36)

        self.create_fond()
        self.create_button()

    def update_textes(self):
        self.textes = [ ["Autonomous Driving with F1/10", ORANGE, self.small, 0, 50],
                        ["Speed :", BLACK, self.small, -30, 150],
                        [str(self.level), BLACK, self.small, 100, 150],
                        ["Status :", BLACK, self.small, -30, 250],
                        ["Destination :", BLACK, self.small, -30, 350]]

    def create_fond(self):
        # Image de la taille de la fenêtre
        self.fond = pygame.Surface((500, 872))
        # En bleu
        self.fond.fill(WHITE)

    def create_button(self):
        self.reset_button = Button(self.fond, "   Reset   ", RED, self.small, -30, 700)
        self.start_button = Button(self.fond, "   Start   ", RED, self.small, -30, 750)
        self.quit_button  = Button(self.fond, "   Quit   ", RED, self.small, -30, 800)
        self.moins_button = Button(self.fond, "  -  ", GREEN, self.small, -30, 600)
        self.plus_button  = Button(self.fond, "  +  ", GREEN, self.small, -30, 650)

    def display_text(self, text, color, font, dx, dy):
        '''Ajout d'un texte sur fond. Décalage dx, dy par rapport au centre.
        '''
        mytext = font.render(text, True, color)  # True pour antialiasing
        textpos = mytext.get_rect()
        textpos.centerx = self.fond.get_rect().centerx + dx
        textpos.centery = dy
        self.fond.blit(mytext, textpos)

    def plus(self):
        self.level += 1
        if self.level == 6: self.level = 5

    def moins(self):
        self.level += -1
        if self.level == 0: self.level = 1

    
    

    def infinite_loop(self):
        while self.loop:
            self.create_fond()
            

            # Boutons
            self.reset_button.display_button(self.fond)
            self.start_button.display_button(self.fond)
            self.quit_button.display_button(self.fond)
            self.moins_button.display_button(self.fond)
            self.plus_button.display_button(self.fond)
            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.reset_button.update_button(self.fond, action=reset)
                    self.start_button.update_button(self.fond, action=start)
                    self.quit_button.update_button(self.fond, action=gamequit)
                    self.moins_button.update_button(self.fond, action=self.moins)
                    self.plus_button.update_button(self.fond, action=self.plus)

            self.update_textes()
            for text in self.textes:
                self.display_text(text[0], text[1], text[2],
                                        text[3], text[4])

            # Ajout du fond dans la fenêtre
            self.screen.blit(self.fond, (0, 0))
            map=pygame.image.load('map.png').convert()
            self.screen.blit(map, (500,0))
            # Actualisation de l'affichage
            pygame.display.update()
            # 10 fps
            clock.tick(10)


def reset():
    print("reset")

def start():
    print("start")

def gamequit():
    print("Quit")
    pygame.quit()
    sys.exit()

def ListenerAudrey():
    rospy.init_node('ListenerAudrey')
    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback)
    rospy.spin()

def callback(msg):
        rospy.loginfo("Received a /cmd_vel message!")
        #rospy.loginfo(rospy.get_caller_id()+"I heard %s", Twist.linear)
        rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
        #rospy.loginfo("Conversion : x lineaire : [%f], x converti: [%f]" %(msg.linear.x, msg.linear.x*10))
        

if __name__ == '__main__':
    game = Game()
    game.infinite_loop()
   