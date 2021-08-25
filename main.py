import cv2
import numpy
# import tensorflow as tf
# import pandas as pd
# import tensorflow_hub as hub



cap = cv2.VideoCapture(0)

width = 512
height = 512

while (1):
    #Capture frame by frame
    _, frame = cap.read()

    blurred = cv2.medianBlur(frame, 15)

    #Convert img to HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    


    #Value for red (To be changed)
    lower_red = numpy.array([60,60,60])
    upper_red = numpy.array([200,200,200])

    #Value for green (Probably fixed)
    lower_green = numpy.array([40, 100, 30])
    upper_green = numpy.array([80, 255, 255])

    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    red_mask = cv2.inRange(hsv, lower_red, upper_red)

    contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours2, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 8000:
            cv2.drawContours(frame, contour, -1, (0,255,0), 3)

    
    for contour2 in contours2:
        area2 = cv2.contourArea(contour2)
        if area2 > 8000:
            cv2.drawContours(frame, contour2, -1, (0,255,0), 3)
    

    cv2.imshow('Original', frame)


    k = cv2.waitKey(10) & 0xFF
    if k == 27:
        break

        

cv2.destroyAllWindows()
cap.release()