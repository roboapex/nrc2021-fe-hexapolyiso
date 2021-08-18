import cv2
import numpy as np

cap = cv2.VideoCapture(1)

while (1):
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([30, 150, 50])
    upper_red = np.array([255, 255, 180])

    lower_green = np.array([40, 100, 30])
    upper_green = np.array([80, 255, 255])

    mask = cv2.inRange(hsv, lower_green, upper_green)
    res = cv2.bitwise_and(frame, frame, mask=mask)


    #kernel = np.ones((5, 5), np.uint8)
    #erosion = cv2.erode(mask, kernel, iterations=1)
    #dilation = cv2.dilate(mask, kernel, iterations=1)

    median = cv2.medianBlur(res, 15)

    #cv2.imshow('Original', frame)
    #cv2.imshow('Mask', mask)
    #cv2.imshow('Erosion', erosion)
    #cv2.imshow('Dilation', dilation)

    cv2.imshow('res', res)
    cv2.imshow('median', median)



    k = cv2.waitKey(10) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()