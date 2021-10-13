try:
    import RPi.GPIO as GPIO
    imported = True
except (RuntimeError, ModuleNotFoundError):
    GPIO = None
    imported = False


from time import sleep

in1 = 24
in2 = 25
in3 = 4
in4 = 5
ena = 23
enb = 26

pa = None
pb = None

def initialze():
    global pa
    global pb

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(in1,GPIO.OUT)
    GPIO.setup(in2,GPIO.OUT)
    GPIO.setup(in3,GPIO.OUT)
    GPIO.setup(in4,GPIO.OUT)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    GPIO.setup(ena,GPIO.OUT)
    GPIO.setup(enb,GPIO.OUT)

    pa=GPIO.PWM(ena,100)
    pb=GPIO.PWM(enb,100)

    pa.start(0)
    pb.start(100)    

def update(vel, ang):
        x = 100 - abs(vel + abs(ang)) 
        if x < 0:
            x = 0

        # print(x)
        
        if vel!=0:
            if ang>0:
                print("turning right")
                GPIO.output(in1,GPIO.HIGH)
                GPIO.output(in2,GPIO.LOW)
                GPIO.output(in3,GPIO.HIGH)
                GPIO.output(in4,GPIO.LOW)
                pa.ChangeDutyCycle(100 - vel)
                pb.ChangeDutyCycle(x)

            elif ang<0:
                print("turning left")
                GPIO.output(in1,GPIO.HIGH)
                GPIO.output(in2,GPIO.LOW)
                GPIO.output(in3,GPIO.HIGH)
                GPIO.output(in4,GPIO.LOW)
                pa.ChangeDutyCycle(x)
                pb.ChangeDutyCycle(100 - vel)

            elif ang==0:
                print("forwards")
                GPIO.output(in1,GPIO.HIGH)
                GPIO.output(in2,GPIO.LOW)
                GPIO.output(in3,GPIO.HIGH)
                GPIO.output(in4,GPIO.LOW)
                pa.ChangeDutyCycle(100 - vel)
                pb.ChangeDutyCycle(100 - vel)  

        elif vel==0:
            if ang<0:
                print("turn on spot left")
                GPIO.output(in1,GPIO.HIGH)
                GPIO.output(in2,GPIO.LOW)
                GPIO.output(in3,GPIO.HIGH)
                GPIO.output(in4,GPIO.LOW)
                pa.ChangeDutyCycle(100-abs(ang))
                pb.ChangeDutyCycle(100)

            elif ang>0:
                print("turn on spot right")
                GPIO.output(in1,GPIO.HIGH)
                GPIO.output(in2,GPIO.LOW)
                GPIO.output(in3,GPIO.HIGH)
                GPIO.output(in4,GPIO.LOW)
                pb.ChangeDutyCycle(100-abs(ang))
                pa.ChangeDutyCycle(100)

            elif ang==0:
                print("stop")
                GPIO.output(in1,GPIO.HIGH)
                GPIO.output(in2,GPIO.LOW)
                GPIO.output(in3,GPIO.HIGH)
                GPIO.output(in4,GPIO.LOW)
                pa.ChangeDutyCycle(100)
                pb.ChangeDutyCycle(100)
        
        else:
            print("<<<  wrong input  >>>")
            print("Velocity or angular velocity undefined")



