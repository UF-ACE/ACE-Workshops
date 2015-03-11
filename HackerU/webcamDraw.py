#!/usr/bin/env python
import cv2

# Get user supplied value of xml file
cascade_path = '/home/peach/Desktop/haarcascade_frontalface_default.xml'
# create cascade and initialize with our xml file
faceCascade = cv2.CascadeClassifier(cascade_path)

# set video source to default webcam
video_cap = cv2.VideoCapture(0)
	
while True:
	# read frame-by-frame ... ret = false when no more frames to read
	ret, frame = video_cap.read()
	# convert frame to grayscale
	frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	# break if it runs out of frames to read
	if frame is None:
		break
	# detect faces
	faces = faceCascade.detectMultiScale(
		frame_gray,
		scaleFactor = 1.15,
		minNeighbors = 5,
		minSize = (10, 10),
		flags = cv2.cv.CV_HAAR_SCALE_IMAGE
	)
	
	# draw rectangle around faces
	for (x, y, w, h) in faces:
		cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 255, 0), 2)
	
	# display new, drawn-over framea
	cv2.imshow('webcam', frame)
	
	key = cv2.waitKey(30)
	if key == 27:
		break

video_cap.release()
cv2.destroyAllWindows()	
