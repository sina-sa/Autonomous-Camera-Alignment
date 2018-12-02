# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

face_cascade = cv2.CascadeClassifier('/home/pi/FYP_Codes/XML_files/front_face_default.xml')

# For each person, one face id
face_id = 1

# Initialize sample face image
count = 0

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	img = frame.array
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3,  minNeighbors=8, minSize=(10, 10))
        
        # Loops for each faces
        for (x,y,w,h) in faces:

            # Crop the image frame into rectangle
            cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)
            
            # Increment sample face image
            count += 1

            # Save the captured image into the datasets folder
            cv2.imwrite('/home/pi/FYP-raspberry-pi/Raspberry-Face-Recognition-master/dataset/User.' + str(face_id) + '.' + str(count) + '.jpg', gray[y:y+h,x:x+w])
            #cv2.imwrite('/home/pi/FYP_Codes/result.jpg',image)

            # Display the video frame, with bounded rectangle on the person's face
            cv2.imshow('frame', img)
        

	key = cv2.waitKey(1) & 0xFF

	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
	# If image taken reach 100, stop taking video
        elif count>30:
            break
