import RPi.GPIO as GPIO
from enum import Enum

from ..interop import *

led_red   = 24
led_yellow = 25
led_green = 4

def shutdown():
  GPIO.cleanup()

def initialze():
  GPIO.setmode(GPIO.BCM)
  GPIO.setwarnings(False)

  GPIO.setup(led_red,GPIO.OUT)
  GPIO.setup(led_yellow,GPIO.OUT)
  GPIO.setup(led_green,GPIO.OUT)

def set_leds(red, yellow, green):
  GPIO.output(led_red,    GPIO.HIGH if red    else GPIO.LOW)
  GPIO.output(led_yellow, GPIO.HIGH if yellow else GPIO.LOW)
  GPIO.output(led_green,  GPIO.HIGH if green  else GPIO.LOW)

def set(status:Status):
  set_leds(status == Status.SEARCH_SAMPLE,
    status == Status.COLLECT_SAMPLE,
    status == Status.SEARCH_LANDER)
