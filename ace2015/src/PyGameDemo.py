#!/usr/bin/env python

# Math
import numpy as np
## Display
import pygame
import time
## Ros
import rospy
## Ros Msgs
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion
from Util import *
from Robot import Robot

# Main class
class PyGameDemo(object): 
	"""
	Class containing methods for working with PyGame and drawing to the screen
	"""
	def __init__(self):

		# init pygame
		pygame.init()

		# dimensions
		self.width = 900
		self.height = 900

		# create the screen
		self.screen = pygame.display.set_mode((self.width, self.height))

		# Create our two Publishers, one for the desired position and one for the 
		# current position
		self.des_pos = rospy.Publisher('/des_pos', PointStamped, queue_size=1)
		self.cur_pos = rospy.Publisher('/cur_pos', PointStamped, queue_size=1)

		# Create our Subscriber, which is subscribed to the /delta_pos topic
		rospy.Subscriber('/delta_pos', PointStamped, self.move_bot)

	def load_background(self):
		"""
		Loads the background image 
		"""
		self.background, self.background_rect = load_image("campsite.jpg")

	def load_robot(self):
		"""
		Creates a new robot, which loads the robot sprite image
		"""
		self.robot = Robot()
		self.robot_sprite = pygame.sprite.RenderPlain((self.robot))

	def publish_desired_pos(self, pos):
		"""
		Publishes the desired position topic of the robot within PyGame 
		"""
		self.des_pos.publish(
			PointStamped(
				header = Header(
						stamp=rospy.Time.now(),
						frame_id='/robot',
				),
				point=Point(x=pos[0],
							y=pos[1],
							z=0
				)
			)
		)

	def publish_current_pos(self, pos):
		"""
		Publishes the current position topic of the robot within PyGame
		"""
		self.cur_pos.publish(
			PointStamped(
				header = Header(
						stamp=rospy.Time.now(),
						frame_id='/robot',
				),
				point=Point(x=pos[0],
							y=pos[1],
							z=0
				)
			)
		)
	
	def move_bot(self, data):
		'''
		Change the robots position via the given data 
		'''
		self.robot.rect.x += data.point.x
		self.robot.rect.y += data.point.y

	def main(self):
		"""
		The application's main method
		"""
		# init my ROS node
		rospy.init_node('PyGameDemo', anonymous=True) 
		
		# load the background		
		self.load_background()
	
		# get the game clock, used for tracking time
		clock = pygame.time.Clock()

		# load teh robot sprite
		self.load_robot()

		# 10 Hz
		rate = rospy.Rate(10)

		# Loop until the process is shut down
		while not rospy.is_shutdown(): 

			# Always publish the robots current position
			pos = (self.robot.rect.x, self.robot.rect.y)
			rospy.loginfo("current position is %s", str(pos))

			# Publishes the current position
			self.publish_current_pos(pos)

			# Check to see if the game has been exited or if a mouse click occurred
	  		for event in pygame.event.get(): 
	  			if event.type is pygame.QUIT:
	  				return
	  			if event.type is pygame.MOUSEBUTTONDOWN:
	  				pt = pygame.mouse.get_pos()
	  				# The location of the mouse click is the new desired position
	  				rospy.loginfo("desired position is %s", str(pt))
	  				# Publish the new desired position 
	  				self.publish_desired_pos(pt)

	  		# Move the game clock forward by 20 ticks
	  		clock.tick(20)
	  		# Redraw the background
	  		self.screen.blit(self.background, self.background_rect)  
	  		# Draw the robot on the screen
	  		self.robot_sprite.draw(self.screen)

	  		# Update the entire contents of the screen
	  		pygame.display.flip()

	  		# Sleeps the process to ensure that this loop is executed only 10 times per second
	  		rate.sleep()

# If this is the main module, execute this code
if __name__ == '__main__':
	try: 
		# Create a new PyGameDemo object
		Demo = PyGameDemo()
		# call PyGameDemo.main() 
		Demo.main()
	except rospy.ROSInterruptException: 
		pass
