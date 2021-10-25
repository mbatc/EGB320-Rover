import RPi.GPIO as GPIO
from time import sleep

en_r = 22
in1_r = 6
in2_r = 5

en_l = 23
in1_l = 24
in2_l = 25

pr = None
pl = None

def shutdown():
    GPIO.cleanup()

def initialze():
    global pr
    global pl

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(in1_r,GPIO.OUT)
    GPIO.setup(in2_r,GPIO.OUT)
    GPIO.setup(in1_l,GPIO.OUT)
    GPIO.setup(in2_l,GPIO.OUT)
    GPIO.setup(en_r,GPIO.OUT)
    GPIO.setup(en_l,GPIO.OUT)
    
    GPIO.output(in1_r, GPIO.LOW)
    GPIO.output(in2_r, GPIO.LOW)
    GPIO.output(in1_l, GPIO.LOW)
    GPIO.output(in2_l, GPIO.LOW)

    pr=GPIO.PWM(en_r,100)
    pl=GPIO.PWM(en_l,100)

    pr.start(0)
    pl.start(100)

def set_motor(speed, in1, in2, en):
    dir = speed < 0
    GPIO.output(in1, GPIO.HIGH if dir else GPIO.LOW)
    GPIO.output(in2, GPIO.LOW  if dir else GPIO.HIGH)
    en.ChangeDutyCycle(abs(speed))

def set_right_motor(speed):
    set_motor(speed, in1_r, in2_r, pr)

def set_left_motor(speed):
    set_motor(speed, in1_l, in2_l, pl)
    
def update(vel, ang):
        x = 100 - abs(vel + abs(ang))

        # print(x)
        
        if vel!=0:
            if ang>0:
                # print("turning right")
                set_left_motor(vel)
                set_right_motor(x)

            elif ang<0:
                # print("turning left")
                set_left_motor(x)
                set_right_motor(vel)

            elif ang==0:
                # print("forwards")
                set_right_motor(vel)
                set_left_motor(vel)

        elif vel==0:
            if ang != 0:
                # print("turn spot right")
                set_left_motor(ang)
                set_right_motor(-ang)

            elif ang==0:
                set_left_motor(0)
                set_right_motor(0)
        
        else:
            print("<<<  wrong input  >>>")
            print("Velocity or angular velocity undefined")



