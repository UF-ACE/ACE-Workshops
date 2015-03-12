#!/usr/bin/env python
import cv2
import sys

# define path to image
imagePath = '/home/peach/Desktop/theOffice.jpg'

# Get user supplied value of xml file
cascPath = '/home/peach/Desktop/haarcascade_frontalface_default.xml'
# Create the haar cascade
faceCascade = cv2.CascadeClassifier(cascPath)

# Read the image
image = cv2.imread(imagePath)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect faces in the image
faces = faceCascade.detectMultiScale(
	gray,
	scaleFactor = 1.15,
	minNeighbors = 5,
	minSize = (10, 10),
	flags = cv2.cv.CV_HAAR_SCALE_IMAGE
) 

# Draw a rectangle around the faces
for (x, y, w, h) in faces:
	cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)



cv2.imshow("Faces found" ,image)
cv2.waitKey(0)

