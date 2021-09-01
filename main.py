import cv2
import numpy

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)


#Value for red (To be changed)
lower_red = numpy.array([0,50,0])
upper_red = numpy.array([10,255,255])

lower_red2 = numpy.array([155,50,0])

upper_red2 = numpy.array([255,255,255])


#Value for green (Probably fixed)
lower_green = numpy.array([40, 50, 40])
upper_green = numpy.array([90, 255, 255])

minsize=2000
while (1):
    _, frame = cap.read()
    blurred = cv2.medianBlur(frame, 15)

    #Convert img to HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    

    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    red_mask = cv2.inRange(hsv, lower_red, upper_red) + cv2.inRange(hsv, lower_red2, upper_red2)
    

    cv2.imshow("redmask",red_mask)
    cv2.imshow("greenmask",green_mask)
    


    contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    contours2, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    try:
        c= max(contours, key = cv2.contourArea)
        if cv2.contourArea(c)>=minsize:
            cv2.drawContours(frame, c, -1, (0,255,0), 3)
            x,y,w,h =cv2.boundingRect(c)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.line(frame,(int(x+(w/2)),0),(int(x+(w/2)),len(blurred)),(0,255,0),1)
    except:
        pass
    try:
        c2=max(contours2, key = cv2.contourArea)
        if cv2.contourArea(c2)>=minsize:
            cv2.drawContours(frame, c2, -1, (0,0,255), 3)
            x2,y2,w2,h2 =cv2.boundingRect(c2)
            cv2.rectangle(frame,(x2,y2),(x2+w2,y2+h2),(0,0,255),2)
            cv2.line(frame,(int(x2+(w2/2)),0),(int(x2+(w2/2)),len(blurred)),(0,0,255),1)
    except:
        pass
    if cv2.contourArea(c2)>cv2.contourArea(c):
        print("red closer",end="\r")
    else:
        print("green closer", end="\r")

    

            
    

    cv2.imshow('Original', frame)


    k = cv2.waitKey(10) & 0xFF
    if k == 27:
        break

        

cv2.destroyAllWindows()
cap.release()
