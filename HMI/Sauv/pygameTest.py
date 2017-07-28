import pygame
from pygame.locals import *

pygame.init()

fenetre=pygame.display.set_mode((640,480))
#fenetre est une surface vide
fond=pygame.image.load('background.jpeg').convert()
#convert convertit limage au bon format

fenetre.blit(fond,(0,0))
#empilement de limage sur la fenetre en partant du coin en haut a gauche


perso=pygame.image.load('patoufet.jpeg').convert()
perso.set_colorkey((255,255,255))
#chargement de limage de patoufet
#convert_alpha permet deviter aue la transparence de limage devienne noire
#Pour les images sans tranparence, on utilise set_colorkey((codecoleur))pour rendre la couleur 
#de fond tranparente

position_perso=perso.get_rect() #rect stocke les coordonnees de la surface


fenetre.blit(perso,position_perso)
#empilement de limage sur la fenetre

ane=pygame.image.load('ane.png').convert()
ane.set_colorkey((255,255,255))
fenetre.blit(ane,(200,300))

fleche=pygame.image.load('fleche.jpg').convert()
fleche=pygame.transform.scale(fleche, (75,75))
fleche=pygame.transform.rotate(fleche, -90)

fleche2 = pygame.image.load('fleche.jpg').convert()
fleche2=pygame.transform.scale(fleche2, (75,75))
fleche2=pygame.transform.rotate(fleche2, +90)

fleche3 = pygame.image.load('fleche.jpg').convert()
fleche3=pygame.transform.scale(fleche3, (90,90))
fleche3=pygame.transform.rotate(fleche3, 0)


#fleche=pygame.transform.chop(fleche,(50,50,300,300))
#fleche=pygame.transform.rotozoom(fleche,0, 2)
#fleche.set_colorkey((255,255,255))
fenetre.blit(fleche,(150,250))
fenetre.blit(fleche2,(250,250))

fenetre.blit(fleche3,(50,250))


car=pygame.image.load('system_car.png').convert()
car=pygame.transform.scale(car, (75,75))
car=pygame.transform.rotate(car, -90)

car2 = pygame.image.load('system_car.png').convert()
car2=pygame.transform.scale(car2, (75,75))
car2=pygame.transform.rotate(car2, +90)

car3 = pygame.image.load('system_car.png').convert()
car3=pygame.transform.scale(car3, (90,90))
car3=pygame.transform.rotate(car3, 0)


#fleche=pygame.transform.chop(fleche,(50,50,300,300))
#fleche=pygame.transform.rotozoom(fleche,0, 2)
#fleche.set_colorkey((255,255,255))
fenetre.blit(car,(150,150))
fenetre.blit(car2,(250,150))

fenetre.blit(car3,(50,150))


#pygame.draw.polygon(fenetre, (0, 0, 0), ((0, 100), (0, 200), (200, 200), (200, 300), (300, 150), (200, 0), (200, 100)))
x=100
y=200
#Arrow=pygame.draw.polygon(fenetre,(0,0,0), ((x+0,y+100),(x+50,y+100),(x+50,y+400), (x+150,y+400),(x+150,y+100),(x+200, y+100), (x+100,y+0)))

pygame.key.set_repeat(400, 30)


pygame.display.flip()
#raifraichissement de lecran pour voir les modifications et limage safficher 
M = [2]
for i in range (len(M)) :
	print('M !')

continuer=1

#for i in range(100):
#	fenetre.blit(fond,(0,0))
#	position_perso=position_perso.move(2,0)
#	fenetre.blit(perso,position_perso)
#	fenetre.blit(ane,(200,300))
#	pygame.display.update()
#	pygame.time.delay(100)


while continuer :
	for event in pygame.event.get(): #on parcourt la liste de tous les evenements recus
		if event.type==QUIT:         #si lun de ces evenements est de type QUIT
			continuer=0              #on arrete la boucle
									 #NB QUIT est une constante de python.locals
		if event.type==KEYDOWN:
			if event.key==K_RETURN:
				print('return return')
			if event.key==K_LEFT:
				position_perso=position_perso.move(0,-3)
			if event.key==K_UP:
				position_perso=position_perso.move(-3,0)
			if event.key==K_RIGHT:
				position_perso=position_perso.move(3,0)
			if event.key==K_DOWN:
				position_perso=position_perso.move(0,+3)
			fenetre.blit(fond,(0,0))
			fenetre.blit(perso,position_perso)
			fenetre.blit(ane,(200,300))
			pygame.display.flip()

		if event.type==MOUSEBUTTONDOWN:
			if event.button==1:
				print('Click click !')
			if event.button == 4:
				print("molette bas", pygame.mouse.get_pos()[1])
			if event.button == 5:
				print("molette haut", pygame.mouse.get_pos()[0])
		if event.type==MOUSEBUTTONDOWN and event.button==3 and event.pos[1]<100:
			print('ZONE DANGEREUSE !')





			




