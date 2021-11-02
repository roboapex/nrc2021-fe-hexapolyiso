

'''
 x | x | o
---+---+---
 o | o | x
---+---+---
 x | o | x



'''









import cv2
import numpy
import board
import adafruit_tcs34725
i2c = board.I2C()
sensor = adafruit_tcs34725.TCS34725(i2c)

wid=320
hei=240



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
        pass
    if redbiggest==-1:
        pass


def checking_state():
    while (1):
        redbiggest = 0
        xcoor = None
        _, frame = cap.read()
        blurred = cv2.medianBlur(frame, 15)

        # Convert img to HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        red_mask = cv2.inRange(hsv, lower_red, upper_red) + cv2.inRange(hsv, lower_red2, upper_red2)

        cv2.imshow("redmask", red_mask)
        cv2.imshow("greenmask", green_mask)

        contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        contours2, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) >= minsize:
                cv2.drawContours(frame, c, -1, (0, 255, 0), 3)
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.line(frame, (int(x + (w / 2)), 0), (int(x + (w / 2)), len(blurred)), (0, 255, 0), 1)
                redbiggest = -1
                xcoor = int(x + (w / 2)) / wid
        if len(contours2) > 0:
            c2 = max(contours2, key=cv2.contourArea)
            if cv2.contourArea(c2) >= minsize:
                cv2.drawContours(frame, c2, -1, (0, 0, 255), 3)
                x2, y2, w2, h2 = cv2.boundingRect(c2)
                cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (0, 0, 255), 2)
                cv2.line(frame, (int(x2 + (w2 / 2)), 0), (int(x2 + (w2 / 2)), len(blurred)), (0, 0, 255), 1)
                redbiggest = 1
                xcoor = int(x2 + (w2 / 2)) / wid
                if cv2.contourArea(c2) > cv2.contourArea(c):
                    print("red closer", end="\r")
                    redbiggest = 1
                    xcoor = int(x2 + (w2 / 2)) / wid
                else:
                    print("green closer", end="\r")
                    redbiggest = -1
                    xcoor = int(x + (w / 2)) / wid
        if redbiggest != 0:
            avoidingstate(redbiggest)

        '''
        if lightsensor sense floor thing
        start turning right
        until sense again then turn other direction
        '''


        cv2.imshow('Original', frame)

        k = cv2.waitKey(10) & 0xFF
        if k == 27:
            break
        

cv2.destroyAllWindows()
cap.release()