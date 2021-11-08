import RPi.GPIO as GPIO
import time

#deg90=0.335

in1 = 14 #change
in2 = 15 #change
in3 = 9 #change
in4=11 #change
en1 = 13
en2 = 12

#INITIALISE 
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)

GPIO.setup(en1,GPIO.OUT)
p=GPIO.PWM(en1,1000)
GPIO.setup(en2,GPIO.OUT)
p2=GPIO.PWM(en2,1000)

#SET SPEED
p.start(60)
p2.start(60)

#MOVEMENT FUNCTIONS
def forward():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
def backward():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
def right():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
def left():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
def stop():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)

def gos(f,s):
    f()
    time.sleep(s)


'''
def movetank(s1,s2,tim):
    p.ChangeDutyCycle(abs(s1))
    p2.ChangeDutyCycle(abs(s2))
    GPIO.output(in1, s1<0)
    GPIO.output(in2, s1>0)
    GPIO.output(in3, s2 < 0)
    GPIO.output(in4, s2 > 0)

    time.sleep(tim)
    stop()
    p.ChangeDutyCycle(60)
    p2.ChangeDutyCycle(60)
    forward()
    time.sleep(0.1)
    stop()
    time.sleep(0.1)
    backward()
    time.sleep(0.1)
    stop()
    time.sleep(0.05)
'''

if __name__== "__main__":
    gos(forward, 1.4)
    gos(right, 0.95)
    gos(forward, 7.5)
    gos(right, 1.2)
    gos(forward, 8)
    gos(right, 1.2)
    gos(forward, 4)
    gos(right, 0.7)
    gos(forward, 5)
    gos(right, 0.8)
    gos(forward, 3)
    stop()
    GPIO.cleanup()
