
import cv2
import numpy as np
import time


frame = cv2.imread('/home/pi/DeepPiCar/driver/data/road1_240x320.png')
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

lower_black = np.array([0, 0, 0])
upper_black = np.array([255, 255, 0])
mask = cv2.inRange(hsv, lower_black, upper_black)

edges = cv2.Canny(mask, 200, 400)
