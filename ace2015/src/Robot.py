from Util import *
import pygame

class Robot(pygame.sprite.Sprite):
	"""
	Our little robot agent sprite class
	"""

	def __init__(self):
		"""
		Update the position of the robot with rect.x and rect.y 
		"""
		pygame.sprite.Sprite.__init__(self)
		self.image, self.rect = load_image('Robot-sprite-1.gif')