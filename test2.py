import RPi.GPIO as GPIO
import time
wind=input("windows").upper()

deg90=0.335

in1 = 14
in2 = 15
in3 = 9
in4=11
en1 = 13
en2 = 12
temp1=1
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(en1,GPIO.OUT)
p=GPIO.PWM(en1,1000)
GPIO.setup(en2,GPIO.OUT)
p2=GPIO.PWM(en2,1000)
p.start(30)
p2.start(30)
def forward():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
def backward():
    GPIO.output(in1,True)
    GPIO.output(in2,False)
    GPIO.output(in3,True)
    GPIO.output(in4,False)
def right():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,True)
    GPIO.output(in4,False)
def left():
    GPIO.output(in1,False)
    GPIO.output(in2,True)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
def stop():
    GPIO.output(in1,False)
    GPIO.output(in2,False)
    GPIO.output(in3,False)
    GPIO.output(in4,False)

stop()
def gos(f,s):
    f()
    time.sleep(s)
    forward()
    time.sleep(0.1)
    stop()
    time.sleep(0.1)
    backward()
    time.sleep(0.1)
    stop()
    time.sleep(0.5)

gos(forward, 5)