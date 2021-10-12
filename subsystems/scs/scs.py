
from ..interop import SCS_ACTION

# Set angles 1
level_1 = 175 #collection system parrelell with ground
up_1 = 60 #collection system in full up position
up_1_travel = 120 #collection system in travel hieght

# Set angle 2
close_2 = 180 #SC fully closed
open_2 = 100 #SC fully closed open 

# Start PWM running on both servos, value of 0 (pulse off)
servo1.start(0)
servo2.start(0)

# Angle cal functions
def Set_angle_1(angle_1):
    duty_1 = angle_1 / 20 + 2
    GPIO.output(11, True)
    servo1.ChangeDutyCycle(duty_1)
    time.sleep(1)
    GPIO.output(11, False)

def Set_angle_2(angle_2):
    duty_2 = angle_2 / 20 + 2
    GPIO.output(12, True)
    servo2.ChangeDutyCycle(duty_2)
    time.sleep(1)
    GPIO.output(12, False)



def CollectSample():
    Set_angle_1(level_1)
    time.sleep(2)
    Set_angle_2(open_2)
    time.sleep(3)
    Set_angle_2(close_2)
    time.sleep(2)
    Set_angle_1(up_1_travel)
    time.sleep(2)
  pass

def DropSample():
    Set_angle_1(level_1)
    time.sleep(2)
    Set_angle_2(open_2)
    time.sleep(2)
    Set_angle_1(up_1_travel)
    time.sleep(2)
    Set_angle_2(close_2)
    time.sleep(2)
  pass

def FlipRock():
    Set_angle_2(close_2)
    time.sleep(1)
    Set_angle_1(level_1)
    time.sleep(4)
    Set_angle_1(up_1)
    time.sleep(4)
    Set_angle_1(up_1_travel)
    time.sleep(2)
  pass

def Perform(action):
  if action == SCS_ACTION.COLLECT_SAMPLE:
    CollectSample()
  elif action == SCS_ACTION.DROP_SAMPLE:
    DropSample()
  elif action == SCS_ACTION.FLIP_ROCK:
    FlipRock()

