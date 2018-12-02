from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
import math
import RPi.GPIO as GPIO


# stepper motor shit
stepper_dirPin = 2
stepper_stepPin = 3
angle_accuracy_of_stepper_motor = 0.45
number_of_turns_stepper_motor = 0
stepper_motor_net_angle = 0

# DC motor shit
DC_INA = 27
DC_INB = 22
DC_EnPin = 17
number_of_turns_DC_motor = 0
direction_of_dc_motor = 0
angle_accuracy_of_dc_motor = 11.25
dc_motor_angle_limit = 157.5
dc_motor_net_angle = 0
dc_motor_position = 0

# encoder shit
IR_pin = 14
encoderPos = 0
dc_motor_position = 0

# intersection
inter_P1x = 0
inter_P1y = 0
inter_P2x = 0
inter_P2y = 0
arm1_Px_old = 0
arm1_Py_old = 16
arm2_Px_old = 0
arm2_Py_old = 0

# final x and y coordinates
width_cm = 0
height_cm = 0
width_cm_old = 0
height_cm_old = 0

# angles that define where each motor needs to go to
angle_arm1 = 0
angle_arm2 = 0
angle_arm1_old = 0
angle_arm2_old = 0

# stops the program if it is 1
running = 1

out_of_frame = 1

# board setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
# stepper motor pins setup
GPIO.setup(stepper_dirPin, GPIO.OUT)
GPIO.setup(stepper_stepPin, GPIO.OUT)
GPIO.output(stepper_dirPin, GPIO.LOW)
GPIO.output(stepper_stepPin, GPIO.LOW)
# DC motor pins setup
GPIO.setup(DC_INA, GPIO.OUT)
GPIO.setup(DC_INB, GPIO.OUT)
GPIO.setup(DC_EnPin, GPIO.OUT)
GPIO.output(DC_INA, GPIO.LOW)
GPIO.output(DC_INB, GPIO.LOW)
GPIO.output(DC_EnPin, GPIO.LOW)
# encoder setup
GPIO.setup(IR_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)



# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (512, 512)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(512, 512))

# allow the camera to warmup
time.sleep(0.1)

# Create Local Binary Patterns Histograms for face recognization
recognizer = cv2.face.createLBPHFaceRecognizer()
# Load the trained mode
recognizer.load('/home/pi/FYP-raspberry-pi/Raspberry-Face-Recognition-master/trainer/trainer.yml')
face_cascade = cv2.CascadeClassifier('/home/pi/FYP_Codes/XML_files/front_face_default.xml')
eye_cascade = cv2.CascadeClassifier('/home/pi/FYP_Codes/XML_files/eyes_default.xml')
# Set the font style
font = cv2.FONT_HERSHEY_SIMPLEX


def read_encoder(channel):
    global encoderPos
    encoderPos += 1
      
# encoder interrupt
GPIO.add_event_detect(IR_pin, GPIO.BOTH, callback=read_encoder)


def stepper_motor_movement(angle):
    global number_of_turns_stepper_motor
    stepper_motor_steps = abs(angle / angle_accuracy_of_stepper_motor) # angle_accuracy_of_stepper_motor
    if angle < 0 :
        GPIO.output(stepper_dirPin, GPIO.HIGH) # Enables the motor to move anti clockwise
        #number_of_turns_stepper_motor = number_of_turns_stepper_motor - (stepper_motor_steps - 1)
    else :
        GPIO.output(stepper_dirPin, GPIO.LOW) # Enables the motor to move clockwise
        #number_of_turns_stepper_motor = number_of_turns_stepper_motor + (stepper_motor_steps - 1)
    print "number_of_turns_stepper_motor "+str(number_of_turns_stepper_motor)
    count = 0
    # Makes 200 pulses for making one full cycle rotation
    while count < stepper_motor_steps :
        GPIO.output(stepper_stepPin, GPIO.HIGH)
        time.sleep(0.005)
        GPIO.output(stepper_stepPin, GPIO.LOW)
        time.sleep(0.005)
        if angle < 0:
            number_of_turns_stepper_motor -= 1
        else:
            number_of_turns_stepper_motor += 1
        count += 1
      
    return;

def dc_motor_movement(angle):
    global direction_of_dc_motor
    global number_of_turns_DC_motor
    global encoderPos
    
    number_turn = round(abs(angle) / angle_accuracy_of_dc_motor)
    if angle < 0: # anti clockwise movement
        GPIO.output(DC_INA, GPIO.HIGH)
        GPIO.output(DC_INB, GPIO.LOW)
        direction_of_dc_motor = 0
    elif angle > 0 : # clockwise movement
        GPIO.output(DC_INA, GPIO.LOW)
        GPIO.output(DC_INB, GPIO.HIGH)
        direction_of_dc_motor = 1
    else :
        GPIO.output(DC_INA, GPIO.LOW)
        GPIO.output(DC_INB, GPIO.LOW)
    while encoderPos < number_turn :
        GPIO.output(DC_EnPin, GPIO.HIGH)
        time.sleep(0.01)
    if direction_of_dc_motor == 1:
        number_of_turns_DC_motor += encoderPos
    else:
        number_of_turns_DC_motor -= encoderPos
    print "number_of_turns_DC_motor "+str(number_of_turns_DC_motor)
    encoderPos = 0;
    GPIO.output(DC_INA, GPIO.HIGH)
    GPIO.output(DC_INB, GPIO.HIGH)
    time.sleep(0.06)
    GPIO.output(DC_INA, GPIO.LOW)
    GPIO.output(DC_INB, GPIO.LOW)
    GPIO.output(DC_EnPin, GPIO.LOW)
    return;

def find_points_of_intersection(x1, y1, x0, y0):
    global inter_P1x
    global inter_P1y
    global inter_P2x
    global inter_P2y
    
    r0 = 16
    r1 = 17
    out_of_frame = 1
    
    d = math.sqrt(math.pow((x1 - x0), 2) + math.pow((y1 - y0), 2))
    a = (math.pow(r0, 2) - math.pow(r1, 2) + math.pow(d, 2))/(2*d)
    if abs(a) > r0:
        a = r0
        out_of_frame = 0
    h = math.sqrt(math.pow(r0, 2) - math.pow(a, 2))
    x2 = x0 + ((a*(x1 - x0))/d)
    y2 = y0 + ((a*(y1 - y0))/d)
    
    inter_P1x = x2 + (h*(y1 - y0)/d)
    inter_P1y = y2 - (h*(x1 - x0)/d)
    inter_P2x = x2 - (h*(y1 - y0)/d)
    inter_P2y = y2 + (h*(x1 - x0)/d)
    return out_of_frame;

def find_angle_of_turn(P1x0, P1y0, P1x1, P1y1, P2x0, P2y0, P2x1, P2y1):
    dx1 = P1x1 - P1x0
    dy1 = P1y1 - P1y0
    dx2 = P2x1 - P2x0
    dy2 = P2y1 - P2y0
    # Calculate the angle to both of the points inorder to sellect the most accurate one.
    ang1 = math.atan2(dy1, dx1)*(180/math.pi)
    ang2 = math.atan2(dy2, dx2)*(180/math.pi)
    angle = ang2 - ang1
    if abs(angle) >= 180 :
        if angle > 0 :
            angle = 360 - angle
        else :
            angle = 360 + angle
    angle = -angle
    return angle;    

def goto_coordinates(X, Y):
    global angle_arm1, angle_arm2
    global arm1_Px_old, arm1_Py_old
    global arm2_Px_old, arm2_Py_old
    global dc_motor_net_angle
    global stepper_motor_net_angle
    global out_of_frame
    global width_cm, height_cm
    global width_cm_old, height_cm_old
    
    print "X "+str(X)
    print "Y "+str(Y)
    out_of_frame = find_points_of_intersection(X, Y, 0, 0)
    if out_of_frame == 1:
        width_cm_old = width_cm
        height_cm_old = height_cm
        arm1_angle1 = find_angle_of_turn(0, 0, 0, 16, 0, 0, inter_P1x, inter_P1y)
        arm1_angle2 = find_angle_of_turn(0, 0, 0, 16, 0, 0, inter_P2x, inter_P2y)

        arm2_angle1 = find_angle_of_turn(inter_P1x, inter_P1y, 0, 0, inter_P1x, inter_P1y, X, Y)
        arm2_angle2 = find_angle_of_turn(inter_P2x, inter_P2y, 0, 0, inter_P2x, inter_P2y, X, Y)
        
        if abs(arm1_angle1) < abs(arm1_angle2):
            angle_arm1 = arm1_angle1
            angle_arm2 = arm2_angle1
            angle_select = 1
        elif abs(arm1_angle1) > abs(arm1_angle2):
            angle_arm1 = arm1_angle2
            angle_arm2 = arm2_angle2
            angle_select = 2
        else:
            if abs(arm2_angle1) < abs(arm2_angle2):
                angle_arm1 = arm1_angle1
                angle_arm2 = arm2_angle1
                angle_select = 1
            else:
                angle_arm1 = arm1_angle2
                angle_arm2 = arm2_angle2
                angle_select = 2
        
        while (abs(angle_arm1) > 360) or (abs(angle_arm2) > 360):
            if angle_arm1 >= 360:
                angle_arm1 -= 360
            elif angle_arm1 <= -360:
                angle_arm1 += 360
            elif angle_arm2 >= 360:
                angle_arm2 -= 360
            elif angle_arm2 <= -360:
                angle_arm2 += 360
            else:
                angle_arm1 = angle_arm1
                angle_arm2 = angle_arm2
              
        if (angle_arm1 != dc_motor_net_angle) or (angle_arm2 != stepper_motor_net_angle) :
            if (angle_arm1 != dc_motor_net_angle) and (angle_arm2 != stepper_motor_net_angle) :
                angle_arm1 -= dc_motor_net_angle
                angle_arm2 -= stepper_motor_net_angle
            elif (angle_arm1 == dc_motor_net_angle) and (angle_arm2 != stepper_motor_net_angle):
                angle_arm1 = 0
                angle_arm2 -= stepper_motor_net_angle
            else:
                angle_arm2 = 0
                angle_arm1 -= dc_motor_net_angle
                
            stepper_motor_net_angle += angle_arm2
            dc_motor_net_angle += angle_arm1  
        else:
            angle_arm1 = 0
            angle_arm2 = 0
                
        print "hight_cm-"+str(height_cm)+"-cm"
        print "width_cm-"+str(width_cm)+"-cm"
        print "inter_P1x-"+str(inter_P1x)
        print "inter_P1y-"+str(inter_P1y)
        print "inter_P2x-"+str(inter_P2x)
        print "inter_P2y-"+str(inter_P2y)
        print "angle_arm1-"+str(angle_arm1)
        print "angle_arm2-"+str(angle_arm2)
        print "dc_motor_net_angle-"+str(dc_motor_net_angle)
        print "stepper_motor_net_angle-"+str(stepper_motor_net_angle)
    else:
        angle_arm1 = 0
        angle_arm2 = 0
    return;

def move_both_motors(angle_arm1, angle_arm2):
    dc_motor_movement(angle_arm1)
    time.sleep(0.1)
    stepper_motor_movement(angle_arm2)
    time.sleep(0.01)
    return;

def processing_video(counter, recognize):
    global img
    global frame
    global width_cm, height_cm
    global width_cm_old, height_cm_old
    global out_of_frame
    
    count = 0
    face_found = 0
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        count += 1
        img = frame.array
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        roi_gray = gray
        roi_color = img
        # Detecting faces in the image
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3,  minNeighbors=8, minSize=(10, 10))
        for (x,y,w,h) in faces:
            face_found = 1
            # Draw a rectangle around the detected face
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            # Draw lines at x and y axis
            cv2.line(img,(256,0),(256,512),(0,255,0),2)
            cv2.line(img,(0,256),(512,256),(0,0,255),2)
            #cv2.imshow('Face', img)
            mid_h = 3*h/5
            mid_w = w/2
            # Choose the region to find the eye
            roi_gray = gray[y:y+mid_h, x:x+mid_w]
            roi_color = img[y:y+mid_h, x:x+mid_w]
            # Detecting 1 of the eyes
            eyes = eye_cascade.detectMultiScale(roi_gray, scaleFactor=1.3, minNeighbors=4, minSize=(7, 7))
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
            #cv2.imshow('Eye 1', roi_color)
            # Choose the region to find the eye
            roi_gray = gray[y:y+mid_h, x+mid_w:x+(mid_w*2)]
            roi_color = img[y:y+mid_h, x+mid_w:x+(mid_w*2)]
            # Detecting the other eye
            eyes = eye_cascade.detectMultiScale(roi_gray, scaleFactor=1.3, minNeighbors=4, minSize=(7, 7))
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
            #cv2.imshow('Eye 2', roi_color)
            if recognize == 1:
                # Recognize the face belongs to which ID
                Id, conf = recognizer.predict(gray[y:y+h,x:x+w])
                print "ID" + str(Id)
                print "conf" + str(conf)
                if conf < 85 :
                    if(Id==1):
                        Id="SINA"
                    elif(Id==2):
                        Id="ZAID"
                else:
                    Id="Unknown"

                # Put text describe who is in the picture
                cv2.putText(img, str(Id), (x,y-40), font, 2, (255,255,255), 3)
            
            height = 256 - (y + (h/2))
            width = 256 - (x + (w/2))
            width_cm = (width * 1.25) / 10
            height_cm = ((height * 1.25) ) / 10
        
        cv2.imshow('Image', img)
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        
        if count == counter:
            break
        
    if (width_cm != width_cm_old) or (height_cm != height_cm_old):
        if (width_cm != width_cm_old) and (height_cm != height_cm_old) :
            width_cm += width_cm_old
            height_cm += height_cm_old
        elif (width_cm == width_cm_old) and (height_cm != height_cm_old) :
            width_cm = width_cm_old
            height_cm += height_cm_old
        else:
            width_cm += width_cm_old
            height_cm = height_cm_old
    else:
        width_cm = width_cm_old
        height_cm = height_cm_old
        
    return face_found;

face_found = processing_video(30, 0)
while running == 1:
    if face_found == 1:
        while running == 1: 
            goto_coordinates(width_cm, height_cm)
            move_both_motors(angle_arm1, angle_arm2)
            
            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("s"):
                running = 0
            processing_video(2, 1)
    else:
        width_cm = 0
        height_cm = 33
        goto_coordinates(width_cm, height_cm)
        move_both_motors(angle_arm1, angle_arm2)
        face_found = processing_video(15, 0)
        if face_found == 0:
            width_cm = 0
            height_cm = -16
            goto_coordinates(width_cm, height_cm)
            move_both_motors(angle_arm1, angle_arm2)
            face_found = processing_video(15, 0)
            if face_found == 0:
                width_cm = 33
                height_cm = 0
                goto_coordinates(width_cm, height_cm)
                move_both_motors(angle_arm1, angle_arm2)
                face_found = processing_video(15, 0)
                if face_found == 0:
                    width_cm = -33
                    height_cm = 0
                    goto_coordinates(width_cm, height_cm)
                    move_both_motors(angle_arm1, angle_arm2)
                    face_found = processing_video(15, 0)
                    if face_found == 0:
                        running = 0
                        
return_angle_dc = number_of_turns_DC_motor * angle_accuracy_of_dc_motor
return_angle_stepper = number_of_turns_stepper_motor * angle_accuracy_of_stepper_motor
move_both_motors(-return_angle_dc, -return_angle_stepper)
#Save the result image
cv2.destroyAllWindows()
