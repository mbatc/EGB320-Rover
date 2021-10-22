
# Import libraries
import RPi.GPIO as GPIO
import time

servo1 = None
servo2 = None

claw_pin = 13
lift_pin = 22

# Set angles 1
level_2= 110 #collection system parrelell with ground
level_rock_2= 100 #collection system parrelell with ground
up_2 = 200 #collection system in full up position
up_2_travel = 130 #collection system in travel hieght
    
# Set angle 2
close_1 = 170 #SC fully closed
open_1 = 100 #SC fully open 
travel_1 = 100
  
def initialize():
    global servo1
    global servo2

    # Set GPIO numbering mode
    GPIO.setmode(GPIO.BCM)
    
    # Set pins 13 & 22 as outputs at 50Hz and as PWM servo1 & servo2
    GPIO.setup(claw_pin,GPIO.OUT)
    servo1 = GPIO.PWM(claw_pin,50) # pin 13 for servo1 (lift)
    GPIO.setup(lift_pin,GPIO.OUT)
    servo2 = GPIO.PWM(lift_pin,50) # pin 22 for servo2 (collect)
    
    # Start PWM running on both servos, value of 0 (pulse off)
    servo1.start(0)
    servo2.start(0)

# Angle cal functions
def Set_angle_1(angle_1):
    duty_1 = angle_1 / 20 + 2
    GPIO.output(lift_pin, True)
    servo1.ChangeDutyCycle(duty_1)
    time.sleep(1)
    GPIO.output(lift_pin, False)

def Set_angle_2(angle_2):
    duty_2 = angle_2 / 20 + 2
    GPIO.output(claw_pin, True)
    servo2.ChangeDutyCycle(duty_2)
    time.sleep(1)
    GPIO.output(claw_pin, False)

#performance functions
def SetToTravel():
    Set_angle_2(up_2_travel)
    time.sleep(0.7)
    Set_angle_1(travel_1)
    time.sleep(0.7)
    
    

def CollectSample_Prepare():
    Set_angle_2(level_2)
    time.sleep(0.7)
    Set_angle_1(open_1)

def CollectSample():
    Set_angle_1(close_1)
    time.sleep(0.7)
    Set_angle_2(up_2_travel)
    time.sleep(0.7)

def DropSample():
    Set_angle_2(level_2)
    time.sleep(0.7)
    Set_angle_1(open_1)
    time.sleep(0.7)
    Set_angle_2(up_2_travel)
    time.sleep(0.7)
    Set_angle_1(close_1)
    time.sleep(0.7)

def FlipRock_Prepare():
    Set_angle_1(close_1)
    time.sleep(0.7)
    Set_angle_2(level_rock_2)

def FlipRock():
    Set_angle_2(up_2)
    time.sleep(0.7)
    Set_angle_2(up_2_travel)
    time.sleep(0.7)

def shutdown():
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
