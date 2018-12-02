from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import sys

# screen size parameters
width = 240
height = 180

# hold xy location in the middle of the face
midFaceX = 0
midFaceY = 0

# middle of the screen
midScreenX = 170
midScreenY = 120

# other variables
count = 0
lastPos = 0
pos = 1
countstop = 0

# cascade path
cascPath = '/home/pi/opencv-3.0.0/data/haarcascades/haarcascade_frontalface_alt.xml'
eyesPath = '/home/pi/opencv-3.0.0/data/haarcascades/haarcascade_eye.xml'

# create haar cascade
faceCascade = cv2.CascadeClassifier(cascPath)
eyesCascade = cv2.CascadeClassifier(eyesPath)

def checkPositionY(midFaceY):
	global pos
	global lastPos
	global countstop
	if midFaceY < midScreenY - 30: #<100
		pos = 1
		print "Face position is upper than the middle screen."
		if pos != lastPos:
			print "Sending serial"
			ser.write('y')
			lastPos = pos

		elif pos == lastPos:
			print "Do nothing"

	elif midFaceY > midScreenY + 30: #>90
		pos = 2
		print "Face position is below than the middle screen."
		if pos != lastPos:
			print "Sending serial"
			ser.write('z')
			lastPos = pos

		elif pos == lastPos:
			print "Do nothing"

	elif (midFaceY >= midScreenY-30 & midFaceY <= midScreenY+30):
		pos = 3
		print "Face position is at the center."
		if pos != lastPos:
			print "Sending serial"
			ser.write('s')
			lastPos = pos

		elif pos == lastPos:
			#print "Do nothing"
			countstop = 1
	return countstop;

def checkPositionX(midFaceX):
	global pos
	global lastPos
	global countstop
	if midFaceX < midScreenX - 30:
		pos = 4
		print "Face position is at the left part of screen."
		if pos != lastPos:
			print "Sending serial"
			ser.write('w')
			lastPos = pos

		elif pos == lastPos:
			print "Do nothing"
	elif midFaceX > midScreenX + 30:
		pos = 5
		print "Face position is at the right part of screen."
		if pos != lastPos:
			print "Sending serial"
			ser.write('x')
			lastPos = pos

		elif pos == lastPos:
			print "Do nothing"
	elif (midFaceX >= midScreenX-30 & midFaceX <= midScreenY+30):
		pos = 3
		print "Face position is at the center."
		if pos != lastPos:
			print "Sending serial"
			ser.write('s')
			lastPos = pos

		elif pos == lastPos:
			#print "Do nothing"
			countstop = 2
	return countstop;

# initialize the camera
camera = PiCamera()
camera.resolution = (240,180)
camera.framerate = 32
camera.hflip = True
rawCapture = PiRGBArray(camera,size=(width,height))

# allow the camera to warmup
time.sleep(0.1)

# capture franes from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

	faces = faceCascade.detectMultiScale(
	gray,
	scaleFactor = 1.1,
	minNeighbors = 5,
	minSize = (30,30),
	flags = cv2.cv.CV_HAAR_SCALE_IMAGE
	)

	#print len(faces)
	
	if (len(faces) == 0):
		servoPanPosition = 90
		servoTiltPosition = 30	

	for (x,y,w,h) in faces:
		cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
		center_point = ((x+(x+w))/2, (y+(y+h))/2)
		cv2.rectangle(image, center_point, center_point, (255,0,0), 2)

		#print midFaceX
		#print midFaceX
		
		midFaceX = x + ((x+w)/2)
		midFaceY = y + ((y+h)/2)

		if count == 0:
                        count = checkPositionY(midFaceY);
		elif count == 1:
                        count = checkPositionX(midFaceX);
	

        center_pointscreen = (width/2, height/2)
	cv2.rectangle(image, center_pointscreen, center_pointscreen, (0,0,255), 2)
	
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate(0)
	
	if key == ord("q"):
		break
	
