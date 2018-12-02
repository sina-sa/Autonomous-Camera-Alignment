import math
import time
import RPi.GPIO as GPIO

# stepper motor shit
stepper_dirPin = 2
stepper_stepPin = 3
angle_accuracy_of_stepper_motor = 0.45
number_of_turns_stepper_motor = 0

# DC motor shit
DC_INA = 27
DC_INB = 22
DC_EnPin = 17
number_of_turns_DC_motor = 0
direction_of_dc_motor = 0
angle_accuracy_of_dc_motor = 11.25
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
    
def read_encoder(channel):
    global encoderPos
    global dc_motor_position
    
    encoderPos += 1
    if direction_of_dc_motor == 0 :
        dc_motor_position -= 1
    else :
      dc_motor_position += 1
      
# encoder interrupt
GPIO.add_event_detect(IR_pin, GPIO.BOTH, callback=read_encoder)

def stepper_motor_movement(angle):
    global number_of_turns_stepper_motor
    stepper_motor_steps = abs(angle / angle_accuracy_of_stepper_motor) # angle_accuracy_of_stepper_motor
    if angle < 0 :
        GPIO.output(stepper_dirPin, GPIO.HIGH) # Enables the motor to move clockwise
        number_of_turns_stepper_motor = number_of_turns_stepper_motor - stepper_motor_steps
    else :
        GPIO.output(stepper_dirPin, GPIO.LOW) # Enables the motor to move anti clockwise
        number_of_turns_stepper_motor = number_of_turns_stepper_motor + stepper_motor_steps
        
    count = 0
    # Makes 200 pulses for making one full cycle rotation
    while count < stepper_motor_steps :
        GPIO.output(stepper_stepPin, GPIO.HIGH)
        time.sleep(0.005)
        GPIO.output(stepper_stepPin, GPIO.LOW)
        time.sleep(0.005)
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
    
    d = math.sqrt(math.pow((x1 - x0), 2) + math.pow((y1 - y0), 2))
    a = (math.pow(r0, 2) - math.pow(r1, 2) + math.pow(d, 2))/(2*d)
    h = math.sqrt(math.pow(r0, 2) - math.pow(a, 2))
    x2 = x0 + ((a*(x1 - x0))/d)
    y2 = y0 + ((a*(y1 - y0))/d)
    
    inter_P1x = x2 + (h*(y1 - y0)/d)
    inter_P1y = y2 - (h*(x1 - x0)/d)
    inter_P2x = x2 - (h*(y1 - y0)/d)
    inter_P2y = y2 + (h*(x1 - x0)/d)
    return;

def find_angle_of_turn(P1x1, P1x0, P1y1, P1y0, P2x1, P2x0, P2y1, P2y0):
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

dc_motor_movement(12)
#time.sleep(1)
stepper_motor_movement(0)
find_points_of_intersection(-14.63, 16.5, 0, 0)
print "P1x", inter_P1x
print "P1y", inter_P1y
print "P2x", inter_P2x
print "P2y", inter_P2y

arm1_angle1 = find_angle_of_turn(0, 0, 16, 0, inter_P1x, 0, inter_P1y, 0)
arm1_angle2 = find_angle_of_turn(0, 0, 16, 0, inter_P2x, 0, inter_P2y, 0)
print "arm1_angle1", arm1_angle1
print "arm1_angle2", arm1_angle2

GPIO.cleanup()