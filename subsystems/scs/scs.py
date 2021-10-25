
# Import libraries
import RPi.GPIO as GPIO
import time
import pigpio

servo1 = None
servo2 = None

claw_pin = 27
lift_pin = 26

lift_flat = 1400
lift_travel = 1650
lift_flip = 1800

claw_open = 1300
claw_close = 2250
claw_travel = 1500

pi = None

def initialize():
    global pi
    pi = pigpio.pi()
    # Set GPIO numbering mode
    GPIO.setmode(GPIO.BCM)

def set_claw(angle):
    Set_angle_2(angle)

def set_lift(angle):
    Set_angle_1(angle)
    
# Angle cal functions
def Set_angle_1(angle_1):
    duty_1 = angle_1 # / 20 + 2

    pi.set_servo_pulsewidth(lift_pin, duty_1)
    time.sleep(0.5)

def Set_angle_2(angle_2):
    duty_2 = angle_2 # / 20 + 2

    pi.set_servo_pulsewidth(claw_pin, duty_2)
    time.sleep(0.5)

#performance functions
def SetToTravel():
    set_lift(lift_travel)
    set_claw(claw_travel)

def CollectSample_Prepare():
    set_lift(lift_flat)
    set_claw(claw_open)

def CollectSample():
    set_claw(claw_close)
    set_lift(lift_travel)

def DropSample():
    set_lift(lift_flat)
    set_claw(claw_open)
    set_lift(lift_flip)
    set_claw(claw_travel)

def FlipRock_Prepare():
    set_lift(lift_flat)
    set_claw(claw_close)
    
def FlipRock():
    set_lift(lift_flip)
    set_claw(claw_open)
    set_lift(lift_travel)

def shutdown():
    pi.stop()
    GPIO.cleanup()
