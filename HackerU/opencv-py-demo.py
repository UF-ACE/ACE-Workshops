# opencv 
import cv2
import time
from multiprocessing import Process

# new videocapture object
cap = cv2.VideoCapture(0)
ServoBlaster = open('/dev/servoblaster', 'w')		
# ServoBlaster is what we use to control the servo motors

# Upper limit in degrees for servo range
_ServoUL = 230
_ServoLL = 75

def forward(): 
		speed = 0.1
		for i in range(_ServoLL, _ServoUL):
			print i
			time.sleep(speed)
			ServoBlaster.write('0=' + str(i) + '\n')	
			ServoBlaster.flush()	
			
def backwards(): 
		speed = 0.1
		for i in range(_ServoUL, _ServoLL, -1): 
			print i
			time.sleep(speed)
			ServoBlaster.write('0=' + str(i) + '\n')	
			ServoBlaster.flush()
			
def servo_test(): 
	while(True): 
		forward()
		time.sleep(0.5)
		backwards()
		time.sleep(0.5)
		
def do_cv(): 			
	while(True):
		# Capture frame-by-frame
		ret, frame = cap.read()

		# Our operations on the frame come here
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		# Display the resulting frame
		cv2.imshow('frame',gray)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	# When everything done, release the capture
	cap.release()
	cv2.destroyAllWindows()

Process(target=do_cv, args=()).start()	# Start the subprocesses

time.sleep(1)

servo_test()