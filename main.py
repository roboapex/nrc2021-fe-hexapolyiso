

'''
 x | x | o
---+---+---
 o | o | x
---+---+---
 x | o | x



'''

'''
# Python Script
# https://www.electronicshub.org/raspberry-pi-l298n-interface-tutorial-control-dc-motor-l298n-raspberry-pi/

import RPi.GPIO as GPIO
from time import sleep

in1 = 24
in2 = 23
en = 25
temp1=1

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
p=GPIO.PWM(en,1000)

p.start(25)
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")

while(1):

    x=input()

    if x=='r':
        print("run")
        if(temp1==1):
         GPIO.output(in1,GPIO.HIGH)
         GPIO.output(in2,GPIO.LOW)
         print("forward")
         x='z'
        else:
         GPIO.output(in1,GPIO.LOW)
         GPIO.output(in2,GPIO.HIGH)
         print("backward")
         x='z'


    elif x=='s':
        print("stop")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        x='z'

    elif x=='f':
        print("forward")
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        temp1=1
        x='z'

    elif x=='b':
        print("backward")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        temp1=0
        x='z'

    elif x=='l':
        print("low")
        p.ChangeDutyCycle(25)
        x='z'

    elif x=='m':
        print("medium")
        p.ChangeDutyCycle(50)
        x='z'

    elif x=='h':
        print("high")
        p.ChangeDutyCycle(75)
        x='z'


    elif x=='e':
        GPIO.cleanup()
        print("GPIO Clean up")
        break

    else:
        print("<<<  wrong data  >>>")
        print("please enter the defined data to continue.....")
        '''







import cv2
import numpy
import time
# import board
# import adafruit_tcs34725
# i2c = board.I2C()
# sensor = adafruit_tcs34725.TCS34725(i2c)

wid=320
hei=240

greentresh=100
redtresh=10

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,wid)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,hei)


#Value for red (To be changed)
lower_red = numpy.array([0,50,0])
upper_red = numpy.array([10,255,255])

lower_red2 = numpy.array([155,50,0])

upper_red2 = numpy.array([255,255,255])


#Value for green (Probably fixed)
lower_green = numpy.array([40, 50, 40])
upper_green = numpy.array([90, 255, 255])



minsize=2000

def avoidingstate(redbiggest):
    if redbiggest==1:
        startturningtime = time.time()
        while True:
            # start turning the right direction
            xcoor = None
            _, frame = cap.read()
            blurred = frame

            # Convert img to HSV
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            red_mask = cv2.inRange(hsv, lower_red, upper_red) + cv2.inRange(hsv, lower_red2, upper_red2)
            cv2.imshow("redmask", red_mask)

            contours2, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            if len(contours2) > 0:
                # print("green")
                c2 = max(contours2, key=cv2.contourArea)
                if cv2.contourArea(c2) >= minsize:
                    cv2.drawContours(frame, c2, -1, (0, 0, 255), 3)
                    x2, y2, w2, h2 = cv2.boundingRect(c2)
                    cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (0, 0, 255), 2)
                    cv2.line(frame, (int(x2 + (w2 / 2)), 0), (int(x2 + (w2 / 2)), len(blurred)), (0, 0, 255), 1)
                    redbiggest = 1
                    xcoor = int(x2 + (w2 / 2)) / wid
            if xcoor < redtresh:
                finalturningtime = time.time() - (startturningtime)

                while True:
                    # start going strainght
                    xcoor = None
                    _, frame = cap.read()
                    blurred = frame

                    # Convert img to HSV
                    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                    red_mask = cv2.inRange(hsv, lower_red, upper_red) + cv2.inRange(hsv, lower_red2, upper_red2)
                    cv2.imshow("redmask", red_mask)

                    contours2, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                    if len(contours2) > 0:
                        # print("green")
                        c2 = max(contours2, key=cv2.contourArea)
                        if cv2.contourArea(c2) >= minsize:
                            cv2.drawContours(frame, c2, -1, (0, 0, 255), 3)
                            x2, y2, w2, h2 = cv2.boundingRect(c2)
                            cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (0, 0, 255), 2)
                            cv2.line(frame, (int(x2 + (w2 / 2)), 0), (int(x2 + (w2 / 2)), len(blurred)), (0, 0, 255), 1)
                            redbiggest = 1
                            xcoor = int(x2 + (w2 / 2)) / wid
                    else:
                        break
                st = time.time()
                while (st + finalturningtime) - time.time() > 0:
                    # turn opposite direction
    if redbiggest==-1:
        startturningtime=time.time()
        while True:
            # start turning the right direction
            xcoor = None
            _, frame = cap.read()
            blurred = frame

            # Convert img to HSV
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            green_mask = cv2.inRange(hsv, lower_green, upper_green)


            cv2.imshow("redmask", red_mask)

            contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)


            if len(contours) > 0:
                # print("red")
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) >= minsize:
                    cv2.drawContours(frame, c, -1, (0, 255, 0), 3)
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.line(frame, (int(x + (w / 2)), 0), (int(x + (w / 2)), len(blurred)), (0, 255, 0), 1)
                    redbiggest = -1
                    xcoor = int(x + (w / 2)) / wid
            if xcoor>greentresh:
                finalturningtime=time.time()-(startturningtime)

                while True:
                    # start going strainght
                    xcoor = None
                    _, frame = cap.read()
                    blurred = frame

                    # Convert img to HSV
                    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                    green_mask = cv2.inRange(hsv, lower_green, upper_green)

                    cv2.imshow("redmask", red_mask)

                    contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

                    if len(contours) > 0:
                        # print("red")
                        c = max(contours, key=cv2.contourArea)
                        if cv2.contourArea(c) >= minsize:
                            cv2.drawContours(frame, c, -1, (0, 255, 0), 3)
                            x, y, w, h = cv2.boundingRect(c)
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            cv2.line(frame, (int(x + (w / 2)), 0), (int(x + (w / 2)), len(blurred)), (0, 255, 0), 1)
                            redbiggest = -1
                            xcoor = int(x + (w / 2)) / wid
                    else:
                        break
                st=time.time()
                while (st+finalturningtime)-time.time()>0:
                    #turn opposite direction



def checking_state():
    while (1):
        # move straight
        redbiggest = 0
        xcoor = None
        _, frame = cap.read()
        blurred = frame

        # Convert img to HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        red_mask = cv2.inRange(hsv, lower_red, upper_red) + cv2.inRange(hsv, lower_red2, upper_red2)

        cv2.imshow("redmask", red_mask)
        cv2.imshow("greenmask", green_mask)

        contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        contours2, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        # print("yes")
        if len(contours) > 0:
            # print("red")
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) >= minsize:
                cv2.drawContours(frame, c, -1, (0, 255, 0), 3)
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.line(frame, (int(x + (w / 2)), 0), (int(x + (w / 2)), len(blurred)), (0, 255, 0), 1)
                redbiggest = -1
                xcoor = int(x + (w / 2)) / wid
        if len(contours2) > 0:
            # print("green")
            c2 = max(contours2, key=cv2.contourArea)
            if cv2.contourArea(c2) >= minsize:
                cv2.drawContours(frame, c2, -1, (0, 0, 255), 3)
                x2, y2, w2, h2 = cv2.boundingRect(c2)
                cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (0, 0, 255), 2)
                cv2.line(frame, (int(x2 + (w2 / 2)), 0), (int(x2 + (w2 / 2)), len(blurred)), (0, 0, 255), 1)
                redbiggest = 1
                xcoor = int(x2 + (w2 / 2)) / wid
                try:
                    if cv2.contourArea(c2) > cv2.contourArea(c):
                        print("red closer", end="\r")
                        redbiggest = 1
                        xcoor = int(x2 + (w2 / 2)) / wid
                    else:
                        print("green closer", end="\r")
                        redbiggest = -1
                        xcoor = int(x + (w / 2)) / wid
                except:
                    pass
        if redbiggest != 0:
            # avoidingstate(redbiggest)
            pass
        '''
        if lightsensor sense floor thing
        start turning right
        until sense again then turn other direction
        '''


        cv2.imshow('Original', frame)

        k = cv2.waitKey(10) & 0xFF
        if k == 27:
            break
        
checking_state()
cv2.destroyAllWindows()
cap.release()