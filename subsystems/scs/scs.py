
# Import libraries
import RPi.GPIO as GPIO
import time
import pigpio

servo1 = None
servo2 = None

claw_pin = 27
lift_pin = 26

# Set angles 1
level_2= 110 #collection system parrelell with ground
level_rock_2= 100 #collection system parrelell with ground
up_2 = 200 #collection system in full up position
up_2_travel = 130 #collection system in travel hieght
    
# Set angle 2
close_1 = 170 #SC fully closed
open_1 = 100 #SC fully open 
travel_1 = 100
  
pi = None

def initialize():
    global pi
    pi = pigpio.pi()
    # Set GPIO numbering mode
    GPIO.setmode(GPIO.BCM)

# Angle cal functions
def Set_angle_1(angle_1):
    duty_1 = angle_1 / 20 + 2

    pi.set_servo_pulsewidth(lift_pin, duty_1)
    time.sleep(0.5)

def Set_angle_2(angle_2):
    duty_2 = angle_2 / 20 + 2

    pi.set_servo_pulsewidth(claw_pin, duty_2)
    time.sleep(0.5)

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
    pi.stop()
    GPIO.cleanup()
