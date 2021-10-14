
# Import libraries
import RPi.GPIO as GPIO
import time

servo1 = None
servo2 = None

# Set angles 1
level_1 = 100 #collection system parrelell with ground
up_1 = 200 #collection system in full up position
up_1_travel = 120 #collection system in travel hieght
    
# Set angle 2
close_2 = 180 #SC fully closed
open_2 = 100 #SC fully open 
travel_2 = 140
  
def initialize():
    global servo1
    global servo2

    # Set GPIO numbering mode
    GPIO.setmode(GPIO.BOARD)
    
    # Set pins 13 & 22 as outputs at 50Hz and as PWM servo1 & servo2
    GPIO.setup(13,GPIO.OUT)
    servo1 = GPIO.PWM(13,50) # pin 13 for servo1 (lift)
    GPIO.setup(22,GPIO.OUT)
    servo2 = GPIO.PWM(22,50) # pin 22 for servo2 (collect)
    
    # Start PWM running on both servos, value of 0 (pulse off)
    servo1.start(0)
    servo2.start(0)

# Angle cal functions
def Set_angle_1(angle_1):
    duty_1 = angle_1 / 20 + 2
    GPIO.output(13, True)
    servo1.ChangeDutyCycle(duty_1)
    time.sleep(1)
    GPIO.output(13, False)

def Set_angle_2(angle_2):
    duty_2 = angle_2 / 20 + 2
    GPIO.output(22, True)
    servo2.ChangeDutyCycle(duty_2)
    time.sleep(1)
    GPIO.output(22, False)

#performance functions
def CollectSample():
    Set_angle_1(level_1)
    time.sleep(2)
    Set_angle_2(open_2)
    time.sleep(3)
    Set_angle_2(close_2)
    time.sleep(2)
    Set_angle_1(up_1_travel)
    time.sleep(2)

def DropSample():
    Set_angle_1(level_1)
    time.sleep(2)
    Set_angle_2(open_2)
    time.sleep(2)
    Set_angle_1(up_1_travel)
    time.sleep(2)
    Set_angle_2(close_2)
    time.sleep(2)

def FlipRock():
    Set_angle_2(close_2)
    time.sleep(1)
    Set_angle_1(level_1)
    time.sleep(4)
    Set_angle_1(up_1)
    time.sleep(4)
    Set_angle_1(up_1_travel)
    time.sleep(2)

def shutdown():
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
