#!/usr/bin/env pythonima
import cv2
import numpy as np
import matplotlib.pyplot as pyplot

def convertToGrayScale(image):
	img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	return img_gray
	
def convertToHSVScale(image):
	img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	return img_hsv
	
def detectFaces(image):
	# Get user supplied value of xml file
	cascPath = '/home/peach/Desktop/haarcascade_frontalface_default.xml'
	# Create the haar cascade
	faceCascade = cv2.CascadeClassifier(cascPath)

	img_gray = convertToGrayScale(image)

	# Detect faces in the image
	faces = faceCascade.detectMultiScale(
		img_gray,
		scaleFactor = 1.15,
		minNeighbors = 5,
		minSize = (10, 10),
		flags = cv2.cv.CV_HAAR_SCALE_IMAGE
	) 

	# Draw a rectangle around the faces
	for (x, y, w, h) in faces:
		cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
	
	return image
	
def main():
	#cap = cv2.VideoCapture(0)
	#cv2.waitkey(0)
	#ret, frame = cap.read()
	#img = frame
	#cv2.imshow('webcam capture', img)
	#cv2.waitKey(0)
	
	# define path to image
	path = '/home/peach/Desktop/theOffice.jpg'
	# load image
	img = cv2.imread(path)
	# display image on screen
	cv2.imshow('original image', img)
	# wait until any key is pressed to continue
	cv2.waitKey(0)
	
	modified_img = convertToGrayScale(img)
	#modified_img = convertToHSVScale(img)
	#modified_img = detectFaces(img)
	
	cv2.imshow('modified image', modified_img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	
	#b,g,r = cv2.split(img) # get b,g,r 
	#rgb_img = cv2.merge([r,g,b]) # switch it to rgb 
	#pyplot.imshow(rgb_img)
	#pyplot.imshow(rgb_img, cmap = pyplot.get_cmap('gray'))
	#pyplot.show()
	#cv2.waitKey(0)
	# then close all windows
	#cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
